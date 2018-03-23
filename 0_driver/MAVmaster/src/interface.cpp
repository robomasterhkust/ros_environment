/**
 * @brief MAVConn class interface
 * @file interface.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <set>
#include <cassert>
#include <cstdio>
#include <console_bridge/console.h>

#include <interface.h>
#include <msgbuffer.h>
#include <serial.h>

namespace mavconn {
#define PFX	"mavconn: "

// static members
std::once_flag MAVConnInterface::init_flag;
std::unordered_map<msgid_t, const mavlink_msg_entry_t*> MAVConnInterface::message_entries {};
std::atomic<size_t> MAVConnInterface::conn_id_counter {0};


MAVConnInterface::MAVConnInterface(uint8_t system_id, uint8_t component_id) :
	sys_id(system_id),
	comp_id(component_id),
	m_status {},
	m_buffer {},
	tx_total_bytes(0),
	rx_total_bytes(0),
	last_tx_total_bytes(0),
	last_rx_total_bytes(0),
	last_iostat(steady_clock::now())
{
	conn_id = conn_id_counter.fetch_add(1);
}

mavlink_status_t MAVConnInterface::get_status()
{
	return m_status;
}

MAVConnInterface::IOStat MAVConnInterface::get_iostat()
{
	std::lock_guard<std::recursive_mutex> lock(iostat_mutex);
	IOStat stat;

	stat.tx_total_bytes = tx_total_bytes;
	stat.rx_total_bytes = rx_total_bytes;

	auto d_tx = stat.tx_total_bytes - last_tx_total_bytes;
	auto d_rx = stat.rx_total_bytes - last_rx_total_bytes;
	last_tx_total_bytes = stat.tx_total_bytes;
	last_rx_total_bytes = stat.rx_total_bytes;

	auto now = steady_clock::now();
	auto dt = now - last_iostat;
	last_iostat = now;

	float dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt).count();

	stat.tx_speed = d_tx / dt_s;
	stat.rx_speed = d_rx / dt_s;

	return stat;
}

void MAVConnInterface::iostat_tx_add(size_t bytes)
{
	tx_total_bytes += bytes;
}

void MAVConnInterface::iostat_rx_add(size_t bytes)
{
	rx_total_bytes += bytes;
}

//Need to modify in order to read MAVlink V1.0
void MAVConnInterface::parse_buffer(const char *pfx, uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
	mavlink_status_t status;
	mavlink_message_t message;

	assert(bufsize >= bytes_received);

	iostat_rx_add(bytes_received);
	for (; bytes_received > 0; bytes_received--) {
		auto c = *buf++;

		// based on mavlink_parse_char()
		auto msg_received = static_cast<Framing>(mavlink_frame_char_buffer(&m_buffer, &m_status, c, &message, &status));
		if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
			_mav_parse_error(&m_status);
			m_status.msg_received = MAVLINK_FRAMING_INCOMPLETE;
			m_status.parse_state = MAVLINK_PARSE_STATE_IDLE;
			if (c == MAVLINK_STX) {
				m_status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
				m_buffer.len = 0;
				mavlink_start_checksum(&m_buffer);
			}
		}

		if (msg_received != Framing::incomplete) {
			log_recv(pfx, message, msg_received);

			if (message_received_cb)
				message_received_cb(&message, msg_received);
		}
	}
}

void MAVConnInterface::log_recv(const char *pfx, mavlink_message_t &msg, Framing framing)
{
	const char *framing_str = (framing == Framing::ok) ? "OK" :
			(framing == Framing::bad_crc) ? "!CRC" :
			(framing == Framing::bad_signature) ? "!SIG" : "ERR";

	const char *proto_version_str = (msg.magic == MAVLINK_STX) ? "v2.0" : "v1.0";

	printf("%s%zu: recv: %s %4s Message-Id: %u [%u bytes] IDs: %u.%u Seq: %u\n",
			pfx, conn_id,
			proto_version_str,
			framing_str,
			msg.msgid, msg.len, msg.sysid, msg.compid, msg.seq);
}

void MAVConnInterface::log_send(const char *pfx, const mavlink_message_t *msg)
{
	const char *proto_version_str = (msg->magic == MAVLINK_STX) ? "v2.0" : "v1.0";

	printf("%s%zu: send: %s Message-Id: %u [%u bytes] IDs: %u.%u Seq: %u\n",
			pfx, conn_id,
			proto_version_str,
			msg->msgid, msg->len, msg->sysid, msg->compid, msg->seq);
}
void MAVConnInterface::set_protocol_version(Protocol pver)
{
	if (pver == Protocol::V10)
		m_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
	else
		m_status.flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
}

Protocol MAVConnInterface::get_protocol_version()
{
	if (m_status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
		return Protocol::V10;
	else
		return Protocol::V20;
}

/**
 * Parse host:port pairs
 */
static void url_parse_host(std::string host,
		std::string &host_out, int &port_out,
		const std::string def_host, const int def_port)
{
	std::string port;

	auto sep_it = std::find(host.begin(), host.end(), ':');
	if (sep_it == host.end()) {
		// host
		if (!host.empty()) {
			host_out = host;
			port_out = def_port;
		}
		else {
			host_out = def_host;
			port_out = def_port;
		}
		return;
	}

	if (sep_it == host.begin()) {
		// :port
		host_out = def_host;
	}
	else {
		// host:port
		host_out.assign(host.begin(), sep_it);
	}

	port.assign(sep_it + 1, host.end());
	port_out = std::stoi(port);
}

/**
 * Parse ?ids=sid,cid
 */
static void url_parse_query(std::string query, uint8_t &sysid, uint8_t &compid)
{
	const std::string ids_end("ids=");
	std::string sys, comp;

	if (query.empty())
		return;

	auto ids_it = std::search(query.begin(), query.end(),
			ids_end.begin(), ids_end.end());
	if (ids_it == query.end()) {
		printf(PFX "URL: unknown query arguments");
		return;
	}

	std::advance(ids_it, ids_end.length());
	auto comma_it = std::find(ids_it, query.end(), ',');
	if (comma_it == query.end()) {
		printf(PFX "URL: no comma in ids= query");
		return;
	}

	sys.assign(ids_it, comma_it);
	comp.assign(comma_it + 1, query.end());

	sysid = std::stoi(sys);
	compid = std::stoi(comp);

	printf(PFX "URL: found system/component id = [%u, %u]", sysid, compid);
}

static MAVConnInterface::Ptr url_parse_serial(
		std::string path, std::string query,
		uint8_t system_id, uint8_t component_id, bool hwflow)
{
	std::string file_path;
	int baudrate;

	// /dev/ttyACM0:57600
	url_parse_host(path, file_path, baudrate, MAVConnSerial::DEFAULT_DEVICE, MAVConnSerial::DEFAULT_BAUDRATE);
	url_parse_query(query, system_id, component_id);

	return std::make_shared<MAVConnSerial>(system_id, component_id,
			file_path, baudrate, hwflow);
}

MAVConnInterface::Ptr MAVConnInterface::open_url(std::string url,
		uint8_t system_id, uint8_t component_id)
{
	/* Based on code found here:
	 * http://stackoverflow.com/questions/2616011/easy-way-to-parse-a-url-in-c-cross-platform
	 */

	const std::string proto_end("://");
	std::string proto;
	std::string host;
	std::string path;
	std::string query;

	auto proto_it = std::search(
			url.begin(), url.end(),
			proto_end.begin(), proto_end.end());
	if (proto_it == url.end()) {
		// looks like file path
		printf(PFX "URL: %s: looks like file path\n", url.c_str());
		return url_parse_serial(url, "", system_id, component_id, false);
	}

	// copy protocol
	proto.reserve(std::distance(url.begin(), proto_it));
	std::transform(url.begin(), proto_it,
			std::back_inserter(proto),
			std::ref(tolower));

	// copy host
	std::advance(proto_it, proto_end.length());
	auto path_it = std::find(proto_it, url.end(), '/');
	std::transform(proto_it, path_it,
			std::back_inserter(host),
			std::ref(tolower));

	// copy path, and query if exists
	auto query_it = std::find(path_it, url.end(), '?');
	path.assign(path_it, query_it);
	if (query_it != url.end())
		++query_it;
	query.assign(query_it, url.end());

	printf(PFX "URL: %s: proto: %s, host: %s, path: %s, query: %s\n",
			url.c_str(), proto.c_str(), host.c_str(),
			path.c_str(), query.c_str());

	if (proto == "serial")
		return url_parse_serial(path, query, system_id, component_id, false);
	else if (proto == "serial-hwfc")
		return url_parse_serial(path, query, system_id, component_id, true);
	else
		throw DeviceError("url", "Unknown URL type");
}

}	// namespace mavconn
