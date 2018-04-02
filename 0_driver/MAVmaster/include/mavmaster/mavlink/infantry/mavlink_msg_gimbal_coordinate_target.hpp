// MESSAGE GIMBAL_COORDINATE_TARGET support class

#pragma once

namespace mavlink {
namespace infantry {
namespace msg {

/**
 * @brief GIMBAL_COORDINATE_TARGET message
 *
 * The target to hit in the camera frame.
 */
struct GIMBAL_COORDINATE_TARGET : mavlink::Message {
    static constexpr msgid_t MSG_ID = 33;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 61;
    static constexpr auto NAME = "GIMBAL_COORDINATE_TARGET";


    float x; /*< Target coordinate, x */
    float y; /*< Target coordinate, y */
    float z; /*< Target coordinate, z */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << x;                             // offset: 0
        map << y;                             // offset: 4
        map << z;                             // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> x;                             // offset: 0
        map >> y;                             // offset: 4
        map >> z;                             // offset: 8
    }
};

} // namespace msg
} // namespace infantry
} // namespace mavlink
