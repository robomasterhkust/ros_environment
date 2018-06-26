cmake_minimum_required(VERSION 2.8.3)
project(can_receive_msg)

find_package(catkin REQUIRED
        COMPONENTS
        message_generation
        std_msgs
        )

add_message_files(DIRECTORY msg
        FILES
        dbus.msg
        gameinfo.msg
        location_xy.msg
        location_zyaw.msg
        power_buffer.msg
        power_shooter_rfid_bufferinfo.msg
        power_vol_cur.msg
        projectile_hlth.msg
		imu_16470.msg
        )

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)
