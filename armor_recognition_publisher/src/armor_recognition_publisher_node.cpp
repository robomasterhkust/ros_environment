//
// Created by beck on 1/2/18.
//

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <cmath>

using namespace cv;
using namespace std;
ros::Publisher pub_cmd;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "armor_recognition_publisher");
    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe("/raw_image", 2, img_callback);

    ros::spin();
}