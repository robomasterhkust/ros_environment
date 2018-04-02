// MESSAGE GIMBAL_SPEED_TARGET support class

#pragma once

namespace mavlink {
namespace infantry {
namespace msg {

/**
 * @brief GIMBAL_SPEED_TARGET message
 *
 * The angular speed target gimbal is expected to rotate at.
 */
struct GIMBAL_SPEED_TARGET : mavlink::Message {
    static constexpr msgid_t MSG_ID = 34;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 138;
    static constexpr auto NAME = "GIMBAL_SPEED_TARGET";


    float yawspeed; /*< Yaw angular speed (rad/s) */
    float pitchspeed; /*< Pitch angular speed (rad/s) */


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
        ss << "  yawspeed: " << yawspeed << std::endl;
        ss << "  pitchspeed: " << pitchspeed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << yawspeed;                      // offset: 0
        map << pitchspeed;                    // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> yawspeed;                      // offset: 0
        map >> pitchspeed;                    // offset: 4
    }
};

} // namespace msg
} // namespace infantry
} // namespace mavlink
