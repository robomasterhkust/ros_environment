/**
 * ROS node to update the detection algorithm with tracking and data association
 * @author Beck Pang
 * @date 2018-11-17
 */
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "rm_cv/vertice.h"

#include "tracking_node.hpp"

#include "../detection/Settings.hpp"
#include "../detection/ArmorDetection.hpp"

using namespace cv;
using namespace std;

ros::Publisher bbox_pub;
string cv_topic;
string bbox_topic;
double tracking_freq = 100;
double detection_freq = 30;
double redetection_freq = 1;

bool tracking_lost = false;
bool armor_detected = false;
bool detection_verified = false;
Rect2d bbox_detect;
Rect2d bbox_detect_prev;
Rect2d bbox_track;

unsigned int tracking_count = 0;

enum _state_detect_and_track_t { NOT_DETECTED, DETECTION, TRACKING };
_state_detect_and_track_t state = NOT_DETECTED;

void
cv_callback(const sensor_msgs::Image::ConstPtr image_ptr) {
    /* code */
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "armor_detect_and_track");
    ros::NodeHandle nh("~");

    nh.param("cv_topic", cv_topic, string("/cam1"));
    nh.param("bbox_topic", bbox_topic, string("/rm_cv/bbox"));

    ros::Subscriber sub_frame = nh.subscribe(cv_topic, 10, cv_callback);
    bbox_pub = nh.advertise<rm_cv::vertice>(bbox_topic, 10);

    ros::Rate rate_track(tracking_freq);
    ros::Rate rate_detect(detection_freq);

    unsigned int tracking_count_max = (unsigned int)(tracking_freq / redetection_freq);

    while (ros::ok()) {
        // state machine
        switch (state) {
            case NOT_DETECTED:
                state = armor_detected ? DETECTION : NOT_DETECTED;
                break;

            case DETECTION:
                if (!armor_detected) { state = NOT_DETECTED; }
                else if (!detection_verified) { state = DETECTION;}
                else { state = TRACKING;}
                break;

            case TRACKING:
                if (tracking_lost) {
                    state = NOT_DETECTED;
                    tracking_count = 0;
                }
                else if (tracking_count >= tracking_count_max) {
                    state = DETECTION;
                    tracking_count = 0;
                }
                else {
                    state = TRACKING;
                    tracking_count++;
                }
                break;
            default: break;
        }

        if (state == NOT_DETECTED) {
            // bbox_detect = detect();
            rate_detect.sleep();
        }
        else if (state == DETECTION) {
            // bbox_detect = detect();
            // detection_verified = data_associate(bbox_detect_prev, bbox_detect);
            rate_detect.sleep();
        }
        else if (state == TRACKING) {
            // bbox_track = track();
            rate_track.sleep();
        }

        ros::spinOnce();
    }
}
