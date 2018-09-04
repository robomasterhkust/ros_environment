#include "CamBase.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <sys/stat.h>
#include <chrono>
#include <ctime>
#include <mutex>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <stdbool.h>

#define CAM_CONFIG_FOLDER_STR "flirConfigs/"

using namespace std;

//shallow copy by default
void FrameInfo::copyTo(FrameInfo &dst)
{
    //perform deep copy
    this->img.copyTo(dst.img);
    dst.rosheader = this->rosheader;
    dst.rotationVec = this->rotationVec;
    dst.translationVec = this->translationVec;
    dst.sourceCamPtr = this->sourceCamPtr;
};

FrameInfo::~FrameInfo(){};

string Camera::getName() const
{
    return config_filename;
};

bool Camera::tryRead()
{
    ros::Time startTime = ros::Time::now();
    if (this->lockcam.try_lock())
    {
        FrameInfo *tempout = getFrame();
        if (tempout)
        {
            clock_gettime(CLOCK_MONOTONIC, &lastRead);
            outQ.enqueue(tempout);
            this->lockcam.unlock();
            ROS_INFO("Camera read from %s took %f ms with latency: %f ms",
                     this->config_filename.c_str(),
                     (ros::Time::now() - startTime).toSec() * 1000.0,
                     (ros::Time::now() - tempout->rosheader.stamp).toSec() * 1000.0);

            return true;
        }
        else
        {
            this->lockcam.unlock();
            return false;
        }
    }
    return false;
};

void Camera::rectifyCoor(cv::Vec3d &crude_Coordinate, cv::Matx33d &cov) const
{
    ROS_INFO("Camera::rectifyCoor : converting");
    ROS_INFO("Camera coor (%f,%f,%f)", crude_Coordinate[0], crude_Coordinate[1], crude_Coordinate[2]);

    cov = inverseRotationMat * cov * inverseRotationMat.t();

    crude_Coordinate = inverseRotationMat * crude_Coordinate - translationVec;
    //ROS_INFO("into gimbal coor (%f,%f,%f)", crude_Coordinate[0], crude_Coordinate[1], crude_Coordinate[2]);
};

const Mat &Camera::getCameraMatrix() const
{
    return cameraMatrix;
};

const Mat &Camera::getDistCoeffs() const
{
    return distCoeffs;
};

bool Camera::applySetting()
{
    bool success = true;
    success &= this->loadAllConfig();
    success &= this->setCamConfig();
    success &= this->getCamConfig();
    success &= this->storeAllConfig();
    return success;
};

bool Camera::loadAllConfig()
{
    FileStorage fs(CAM_CONFIG_FOLDER_STR + config_filename, cv::FileStorage::READ);
    if (fs.isOpened())
    {
        loadBaseParameters(fs);
        loadDriverParameters(fs);
        fs.release();
        ROS_INFO("Camera load from %s success", config_filename.c_str());
        return true;
    }

    ROS_INFO("Camera load from %s failed", config_filename.c_str());
    return false;
};

bool Camera::storeAllConfig()
{
    FileStorage fsr(CAM_CONFIG_FOLDER_STR + config_filename, cv::FileStorage::READ);
    int driverType = 0;
    fsr["driverType"] >> driverType;
    fsr.release();

    FileStorage fs(CAM_CONFIG_FOLDER_STR + config_filename, cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "driverType" << driverType;
        storeBaseParameters(fs);
        storeDriverParameters(fs);
        fs.release();
        ROS_INFO("Camera store to %s success, driver type %d", config_filename.c_str(), driverType);
        return true;
    }
    else
    {
        fs.release();
        ROS_INFO("Camera store to %s failed", config_filename.c_str());
        return false;
    }
};

bool Camera::loadBaseParameters(const FileStorage &fs)
{
    cv::Vec3f tempRV;
    const FileNode &node = fs["CameraBase"];
    fsHelper::readOrDefault(node["cameraMatrix"], cameraMatrix);
    fsHelper::readOrDefault(node["distCoeffs"], distCoeffs);
    fsHelper::readOrDefault(node["rotationVec"], rotationVec, {0, 0, 0});
    fsHelper::readOrDefault(node["translationVec"], translationVec, {0, 0, 0});
    fsHelper::readOrDefault(node["minReadDelay_ms"], minReadDelay_ms, 0);
    fsHelper::readOrDefault(node["maxReadDelay_ms"], maxReadDelay_ms, 100);

    //calculate the valur of rotaiton matrix and its inverse
    cv::Rodrigues(rotationVec, rotationMat);
    invert(rotationMat, inverseRotationMat);
};

bool Camera::storeBaseParameters(FileStorage &fs)
{
    fs << "CameraBase"
       << "{"
       << "cameraMatrix" << cameraMatrix
       << "distCoeffs" << distCoeffs
       << "rotationVec" << rotationVec
       << "translationVec" << translationVec
       << "minReadDelay_ms" << minReadDelay_ms
       << "maxReadDelay_ms" << maxReadDelay_ms
       << "}";
};

bool Camera::solvePNP(cv::InputArray objectPoints, const vector<Point2f> &imagePoints,
                      cv::OutputArray rvec, cv::OutputArray tvec,
                      bool useExtrinsicGuess, int flags) const
{
    return cv::solvePnP(objectPoints, imagePoints, this->cameraMatrix, this->distCoeffs, rvec, tvec, useExtrinsicGuess, flags);
};

void Camera::showSettingWindow()
{

    namedWindow(this->config_filename + " control panel");
};
