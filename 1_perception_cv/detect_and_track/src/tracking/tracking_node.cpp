/**
 * ROS node to update the detection algorithm with tracking and data association
 * @author Beck Pang
 * @date 2018-11-17
 */
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "rm_cv/vertice.h"

#include "tracking_node.hpp"
#include "armor_detect.h"
#include "detection_setting.hpp"

using namespace cv;
using namespace std;

ros::Publisher bbox_pub;
string cv_topic;
string bbox_topic;
double tracking_freq = 80;
double detection_freq = 30;
double redetection_freq = 1;

bool image_received = false;

Rect2d bbox_detect;
Rect2d bbox_detect_prev;

enum _detect_state_t { IDLE, DETECTION, ASSOCIATION, TRACKING };
_detect_state_t state = IDLE;


// image tracker
vector<Mat> image_stack;
Ptr<Tracker> tracker;

Settings settings("settings.xml");

bool
detect(const Mat cur_frame, Rect2d &bbox) {
    LightFilterSetting *lightSetting = new LightFilterSetting();

    LightStorage* lights = LightFinder::findLight(cur_frame, lightSetting, &settings);

//    vector<ArmorProcessor::LightGp> RLightGps;
//    vector<ArmorProcessor::LightGp> BLightGps;
//    armorGrouper(lights->lightsR, RLightGps);
//    armorGrouper(lights->lightsB, BLightGps);
    return false;
}

void
kalman_filter_init(const Rect2d &bbox) {

}

bool
kalman_filter_update(const Rect2d &bbox) {
    return false;
}

void
kalman_filter_clear() {

}

void
init_tracker(const Mat &cur_frame, const Rect2d &bbox) {
    tracker = TrackerKCF::create();
    tracker->init(cur_frame, bbox);
}

/* TODO: Add 2D feature correspondence
    bool feature_correspondence
 */

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

    // For state machine
    unsigned int verify_count = 0;
    unsigned int tracking_count = 0;
    unsigned int verify_count_max = 5;
    unsigned int tracking_count_max = (unsigned int)(tracking_freq / redetection_freq);

    settings.load();

    while (ros::ok()) {
        // state machine
        switch (state) {
            case IDLE: {
                state = image_received ? DETECTION : IDLE;
                break;
            }
            case DETECTION: {
                bool armor_detected = false;
                // Action
                if (image_received) {
                    armor_detected = detect(image_stack.front(), bbox_detect);
                }

                // Change state condition
                if (!image_received)
                    state = IDLE;
                else if (!armor_detected)
                    state = DETECTION;
                else
                    state = ASSOCIATION;

                // Describe the state edge
                switch (state) {
                    case IDLE:          break;
                    case DETECTION:     break;
                    case ASSOCIATION:
                        verify_count = 0;
                        kalman_filter_init(bbox_detect);
                        break;
                    case TRACKING:      break;
                }
                break;
            }
            case ASSOCIATION: {
                bool armor_detected = false;
                bool detection_verified = false;
                bool count_pass = false;

                // Action
                if (image_received) {
                    armor_detected = detect(image_stack.front(), bbox_detect);
                    if (armor_detected) {
                        detection_verified = kalman_filter_update(bbox_detect);
                        if (detection_verified) {
                            verify_count++;
                            count_pass = !(verify_count < verify_count_max);
                        }
                    }
                }

                // Change state condition
                if (!image_received)
                    state = IDLE;
                else if (!armor_detected || !detection_verified )
                    state = DETECTION;
                else if (!count_pass)
                    state = ASSOCIATION;
                else
                    state = TRACKING;

                // Describe the state edge
                switch (state) {
                    case IDLE:          kalman_filter_clear(); break;
                    case DETECTION:     kalman_filter_clear(); break;
                    case ASSOCIATION:   break;
                    case TRACKING:
                        tracking_count = 0;
                        init_tracker(image_stack.front(), bbox_detect);
                        break;
                }
                break;
            }
            case TRACKING: {
                bool tracking_ok = false;
                bool count_pass = false;

                // Action
                if (image_received) {
                    tracking_ok = tracker->update(image_stack.front(), bbox_detect);
                    if (tracking_ok) {
                        tracking_count++;
                        count_pass = !(tracking_count < tracking_count_max);
                    }
                }

                // Change state condition
                if (!image_received)
                    state = IDLE;
                else if (!tracking_ok)
                    state = DETECTION;
                else if (count_pass)
                    state = ASSOCIATION;
                else
                    state = TRACKING;

                switch (state) {
                    case IDLE:          break;
                    case DETECTION:     break;
                    case ASSOCIATION:   break;
                    case TRACKING:
                        verify_count = 0;
                        kalman_filter_init(bbox_detect);
                        break;
                }
                break;
            }
        }

        // control update rate
        switch (state) {
            case IDLE:
            case DETECTION:
            case ASSOCIATION:
                rate_detect.sleep();
                break;
            case TRACKING:
                rate_track.sleep();
                break;
        }
        ros::spinOnce();
    }
}
