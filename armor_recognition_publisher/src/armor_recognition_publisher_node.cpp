//
// Created by beck on 1/2/18.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include "../include/sort.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
ros::Publisher pub_cmd;

typedef vector<Point> Contour;

char winName[20] = "Live";
Mat frame;
VideoCapture cap;
int iLowH = 110;
int iHighH = 140;
int iLowS = 90;
int iHighS = 250;
int iLowV = 90;
int iHighV = 255;

int RiLowH = 100;
int RiHighH = 140;
int RiLowS = 90;
int RiHighS = 255;
int RiLowV = 90;
int RiHighV = 255;

int extime = 10;
vector<Contour> bluelight;
vector<Contour> redlight;
int minContourSize = 20;

void recConB(Mat input)
{
    bluelight.clear();

    Mat clone;
    cvtColor(input, clone, CV_RGB2GRAY);
    threshold(clone, clone, 100, 255, THRESH_BINARY);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;


    Mat tempHSV;
    vector<Mat> hsvSplit;
    cvtColor(input, tempHSV, COLOR_BGR2HSV);
    split(tempHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, tempHSV);
    Mat imgThresholded;
    inRange(tempHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
//    imshow("HSV", imgThresholded);
    GaussianBlur(imgThresholded, imgThresholded, Size(5, 5), 0, 0);
//    imshow("Gaussian", imgThresholded);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(imgThresholded, imgThresholded, element);
//    imshow("Dilate", imgThresholded);

    findContours(imgThresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    cout << "contour.size: " << contours.size() << endl;
    vector<Rect> boundRect(contours.size());

    for (int i = 0; i < contours.size(); i++) {
        cout << "ContourSize: " << contours[i].size() << endl;
        boundRect[i] = boundingRect(contours[i]);
        Mat temp = input(boundRect[i]);
        Scalar mean = cv::mean(temp);
        if (mean[0] > 30) {
            bluelight.push_back(contours[i]);
        }
    }
    for (int i = 0; i < bluelight.size(); i++) {
        rectangle(input, boundingRect(bluelight[i]), Scalar(0, 0, 255), 2);
    }
//    imshow("frameB", input);
    waitKey(10);
}

void recConR(Mat input)
{
    redlight.clear();
    Mat clone;
    cvtColor(input, clone, CV_RGB2GRAY);
    threshold(clone, clone, 100, 255, THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 1));
    dilate(clone, clone, element);
//    imshow("Dilate", clone);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(clone, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    cout << "contour.size: " << contours.size() << endl;
    vector<Rect> boundRect(contours.size());

    for (int i = 0; i < contours.size(); i++) {
        boundRect[i] = boundingRect(contours[i]);
        Mat temp = input(boundRect[i]);
        Mat tempHSV;
        vector<Mat> hsvSplit;
        cvtColor(temp, tempHSV, COLOR_BGR2HSV);
        split(tempHSV, hsvSplit);
        equalizeHist(hsvSplit[2], hsvSplit[2]);
        merge(hsvSplit, tempHSV);
        Mat imgThresholded;
        inRange(tempHSV, Scalar(RiLowH, RiLowS, RiLowV), Scalar(RiHighH, RiHighS, RiHighV), imgThresholded);
        Scalar mean = cv::mean(imgThresholded);
        if (mean[0] > 100) {
            redlight.push_back(contours[i]);
        }
    }
    for (int i = 0; i < redlight.size(); i++) {
        rectangle(input, boundingRect(redlight[i]), Scalar(0, 0, 255), 2);
    }
    imshow("frameR", input);
    waitKey(10);
}

void printCenter(vector<Contour> lights)
{
    vector<Pair *> pairs;
    vector<Pair *> paired;
    vector<Contour>::iterator pc = lights.begin();
    for (; pc < lights.end(); pc++) {
        if (contourArea(*pc) < minContourSize)
            pc = lights.erase(pc);
    }
    for (pc = lights.begin(); pc < lights.end(); pc++) {
        if (pc == lights.begin()) {
            Pair *temp = new Pair;
            temp->addRect(*pc);
            pairs.push_back(temp);
            continue;
        }
        vector<Pair *>::iterator pp = pairs.begin();
        bool matcha = false;
        for (; pp < pairs.end(); pp++) {
            if ((*pp)->match(*pc)) {
                matcha = true;
                break;
            }
        }
        if (matcha) {
            continue;
        }

        Pair *temp = new Pair;
        temp->addRect(*pc);
        pairs.push_back(temp);
        continue;
    }
    vector<Pair *>::iterator pp = pairs.begin();

    for (vector<Pair *>::iterator pp = pairs.begin(); pp < pairs.end(); pp++) {
        if ((*pp)->isCompeleted) {
            paired.push_back(*pp);
        } else {
            delete *pp;
        }
    }
    if (paired.empty()) {
        cout << " no valid center" << endl;
        // transmit center(empty) here
    } else {
        vector<Pair *>::iterator p = paired.begin();
        vector<Pair *>::iterator max = p;
        for (; p < paired.end(); p++) {
            if ((*p)->size >= (*max)->size) {
                max = p;
            }
        }
        cout << (*max)->center.x << "      " << (*max)->center.y << endl;
        //  transmit center here
        geometry_msgs::Twist cv_output;
        cv_output.linear.x = (*max)->center.x;
        cv_output.linear.y = (*max)->center.y;
        pub_cmd.publish(cv_output);
    }
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    Mat frame;
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

    frame = bridge_ptr->image;

    recConB(frame);

    printCenter(bluelight);

    waitKey(50);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "armor_recognition_publisher");
    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe("/cv_camera/image_raw", 2, img_callback);
    pub_cmd = n.advertise<geometry_msgs::Twist>("/cv_result/pixel", 10);

    ros::spin();
}