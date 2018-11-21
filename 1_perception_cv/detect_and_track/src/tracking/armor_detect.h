//
// Created by beck on 21/11/18.
//

#ifndef RM_CV_ARMOR_DETECT_H
#define RM_CV_ARMOR_DETECT_H

#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <string>
#include <vector>
#include "detection_setting.hpp"

using namespace std;
using namespace cv;

class Light
{
public:
    /**
     * @brief Construct a new Light object
     * the rect will be smaller in dimension by 1 pixel
     */
    Light(cv::RotatedRect &_rect, vector<cv::Point> &contour);

    cv::RotatedRect rect;

    cv::Point2f vertex[2]; // 0 is upper, 1 is lower

    /**
     * @brief test if the tool body belongs to this light, and update this light with tool,
     * tool should be deleted after the merging
     * the result of A.merge(B) should be same with B.merge(A)
     */
    bool merge(const Light &other);

    double length() const;
};

class LightStorage
{
public:
    LightStorage(const Mat &_preprocessedImgR,
                 const Mat &_preprocessedImgB)
            : preprocessedImgR(_preprocessedImgR),
              preprocessedImgB(_preprocessedImgB)
            {};

    void joinBrokenLights();

    void drawLights(Mat &img);

    Mat preprocessedImgR;
    Mat preprocessedImgB;
    Mat preprocessedImgOR;

    vector<Light> lightsB;
    vector<Light> lightsR;

    Settings *settings;

    const Scalar redLightDrawColor = Scalar(0, 255, 255);
    const Scalar blueLightDrawColor = Scalar(255, 255, 0);
    const Scalar redArmorDrawColor = Scalar(0, 0, 255);
    const Scalar blueArmorDrawColor = Scalar(255, 0, 0);
    const Scalar lightGPDrawColor = Scalar(255, 255, 255);
};

namespace LightFinder
{
    LightStorage *findLight(Mat image, LightFilterSetting *setting, Settings *armor_setting);

    static void ConvertRRectAngle2Normal(RotatedRect &rRect);
    static bool testAspectRatio(const RotatedRect &light, Settings *armor_setting);
    static bool testArea(const RotatedRect &light, Settings *armor_setting);
    static bool testTilt(const RotatedRect &light, Settings *armor_setting);
}

#endif //RM_CV_ARMOR_DETECT_H
