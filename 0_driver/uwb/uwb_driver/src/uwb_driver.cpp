#include <uwb_msgs/uwb.h>
#include <string>
#include <ros/ros.h>
#include <math.h>

#define MAX_STATION_NUM     6
#define EPSILON             std::numeric_limits<double>::epsilon()

ros::Publisher uwb_publisher;
bool uwb_start = false;
int uwb_index = 0;
double temp_pos_x = 0;
double temp_pos_y = 0;
double temp_pos_theta = 0;
double temp_distance[MAX_STATION_NUM];

uint16_t temp_error = 0;

void msgCallback(const can_msgs::Frame &f) {
    if (uwb_start) {
        if (f.dlc == 8) {
            uwb_index++;
            if (uwb_index == 1) {
                temp_pos_x =
                        ((int16_t)((uint16_t) f.data[1] << 8 | (uint16_t) f.data[0])) / 100.0; // in meter
                temp_pos_y =
                        ((int16_t)((uint16_t) f.data[3] << 8 | (uint16_t) f.data[2])) / 100.0; // in meter
                temp_pos_theta =
                        (uint16_t)((uint16_t) f.data[5] << 8 | (uint16_t) f.data[4]) / (2 * M_PI * 100); // in rad
                temp_distance[0] = ((int16_t)((uint16_t) f.data[7] << 8 | (uint16_t) f.data[6])) / 100.0; // in meter
            } else if (uwb_index == 2) {
                temp_distance[1] = ((int16_t)((uint16_t) f.data[1] << 8 | (uint16_t) f.data[0])) / 100.0; // in meter
                temp_distance[2] = ((int16_t)((uint16_t) f.data[3] << 8 | (uint16_t) f.data[2])) / 100.0; // in meter
                temp_distance[3] = ((int16_t)((uint16_t) f.data[5] << 8 | (uint16_t) f.data[4])) / 100.0; // in meter
                temp_distance[4] = ((int16_t)((uint16_t) f.data[7] << 8 | (uint16_t) f.data[6])) / 100.0; // in meter
            }
        } else if (f.dlc == 6) {
            uwb_index = 0;

            temp_distance[5] = ((int16_t)((uint16_t) f.data[1] << 8 | (uint16_t) f.data[0])) / 100.0; // in meter
            temp_error = (uint16_t)((uint16_t) f.data[2] << 8 | (uint16_t) f.data[1]);

            1 == ((num >> bit) & 1);

            uwb_msgs::uwb uwb_msg;
            uwb_msg.header.stamp = f.header.stamp;
            uwb_msg.header.frame_id = "world";
            uwb_msg.pos_x = temp_pos_x;
            uwb_msg.pos_y = temp_pos_y;
            uwb_msg.pos_theta = temp_pos_theta;
            uwb_msg.distance = temp_distance;
            uwb_msg.num = 0;
            for (int i = 0; i < MAX_STATION_NUM; i++) {
                if (temp_distance[i] > EPSILON)
                    uwb_msg.num++;
            }
            uwb_msg.check_failed = ((temp_error >> 3) & 1);
            uwb_msg.need_calibrated = ((temp_error >> 2) & 1);
            uwb_msg.level = (uint8_t)()
        }
    }
    if (f.dlc == 6 && !uwb_start) {
        uwb_start = true;
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "uwb_driver_node");
    ros::NodeHandle nh("~");
    uwb_publisher = nh.advertise<nav_msgs::Odometry>("/uwb_odom", 100);
    ros::Subscriber can_subscriber = nh.subscribe("/received_messages", 100, msgCallback);

    ros::spin();
}
