#include "defines.hpp"
#include "Settings.hpp"
#include "Camera.hpp"
#include "V4LCamDriver.hpp"
#include "VideoIn.hpp"
#include "StopWatch.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <sys/stat.h>
#include <chrono>
#include <ctime>
#include <mutex>
#include "ros/ros.h"

#ifdef USE_MVSDK
#include "mvCamera.hpp"
#endif

using namespace std;

FrameInfo::FrameInfo(const FrameInfo &target)
{
    //perform deep copy
    img = target.img;
    capTime = target.capTime;
    rotationVec = target.rotationVec;
    translationVec = target.translationVec;
    sourceCamPtr = target.sourceCamPtr;
};

bool Camera::tryRead()
{
    if (this->lockcam.try_lock())
    {
        clock_gettime(CLOCK_MONOTONIC, &lastRead);
        FrameInfo *tempout = getFrame();
        if (tempout)
        {
            outQ[0].enqueue(tempout);
            for (int i = 1; i < outQCount; i++)
            {
                outQ[i].enqueue(new FrameInfo(*tempout));
            }

            this->lockcam.unlock();
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

void Camera::rectifyCoor(cv::Vec3d &crude_Coordinate) const
{
#ifdef DEBUG
    ROS_INFO("Converting camera coordinate to gun coordinate");
    cout << "Camera coor:" << crude_Coordinate << endl;
#endif
    Mat x(3, 1, CV_32F);
    x.at<float>(0, 0) = crude_Coordinate[0];
    x.at<float>(1, 0) = crude_Coordinate[1];
    x.at<float>(2, 0) = crude_Coordinate[2];

    // cout << "inverseRotationMat " << inverseRotationMat << endl
    //      << "t vec  translationVec" << endl;
    Mat temp = inverseRotationMat * x - translationVec;

    crude_Coordinate[0] = temp.at<float>(0, 0);
    crude_Coordinate[1] = temp.at<float>(1, 0);
    crude_Coordinate[2] = temp.at<float>(2, 0);
#ifdef DEBUG
    cout << "Gun coor:" << crude_Coordinate << endl;
#endif
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
    success &= this->closeStream();
    success &= this->loadAllConfig();
    success &= this->writeCamConfig();
    success &= this->readCamConfig();
    success &= this->storeAllConfig();
    success &= this->startStream();
    return success;
};

void tryReadCam(const Settings &settings, vector<Camera *> &cams)
{
    timespec uptime;
    clock_gettime(CLOCK_MONOTONIC, &uptime);
    int tempReadDelay[cams.size()];
    for (int i = 0; i < cams.size(); i++)
    {
        tempReadDelay[i] = (uptime.tv_sec - cams[i]->lastRead.tv_sec) * 1000 +
                           (uptime.tv_nsec - cams[i]->lastRead.tv_nsec) / 1000000;
        if (tempReadDelay[i] > cams[i]->maxReadDelay_ms)
        {
            cams[i]->tryRead();
            return;
        }
    }
    for (int i = 0; i < cams.size(); i++)
    {
        if (tempReadDelay[i] > cams[i]->minReadDelay_ms)
        {
            cams[i]->tryRead();
            return;
        }
    }
};

/**
 * @brief initiallize a camera object form the specified file name, and check if the camera is accessable
 * 
 * @param filename 
 * @param _outQCount the number of output queues that the camera objects should copy image to 
 * @return Camera* NULL if the camera is not available, otherwise point to the camera object
 */
Camera *startCamFromFile(const string &filename, int _outQCount)
{
    Camera *tempCamera = NULL;
    if (fileExist("CamConfigs/" + filename))
    {
        ROS_INFO("Cam Config file CamConfigs\\%s found", filename.c_str());
        FileStorage fs("CamConfigs/" + filename, cv::FileStorage::READ);
        int driverType;
        fsHelper::readOrDefault(fs["driverType"], driverType, 0);
        switch (driverType)
        {
        case 0:
            ROS_INFO("creating V4L camera object");
            tempCamera = new V4LCamDriver(_outQCount, filename);
            break;

#ifdef USE_MVSDK
        case 1:
            ROS_INFO("creating mvux camera object");
            tempCamera = new mvCamera(_outQCount, filename);
            break;
#endif
        }
        fs.release();

        if (tempCamera)
        {
            tempCamera->loadAllConfig();
            tempCamera->storeAllConfig();
            if (tempCamera->initialize())
            {
                if (tempCamera->startStream())
                {
                    ROS_INFO("Sucessfully created camera object from %s", filename.c_str());
                    return tempCamera;
                }
                else
                {
                    ROS_INFO("Camera object from %s startStream failed", filename.c_str());
                }
            }
            else
            {
                ROS_INFO("Camera object from %s initialize failed", filename.c_str());
            }
            ROS_INFO("Camera object from %s cannot be started", filename.c_str());
        }
        else
        {
            //device unsupported, or config file broken
            ROS_INFO("Camera in %s unsupported, or file is broken", filename.c_str());
        }
    }
    else
    {
        ROS_INFO("Cam Config file CamConfigs\%s not found\n", filename.c_str());
    }
    return NULL;
}

/**
 * @brief Create a Cams object
 * 
 * @param settings the data read from the setting file
 * @param cams the vector of pointers to camera objects
 * @param _outQCount 
 */
void startCams(const Settings &settings, vector<Camera *> &cams, int _outQCount)
{
    //create the CamConfigs folder if there is not one
    struct stat buffer;
    if (stat("CamConfigs/", &buffer) == -1)
    {
        mkdir("CamConfigs/", ACCESSPERMS);
    }
    for (auto i : cams)
        delete i;
    cams.clear();
    for (int i = 0; i < MAX_CAM_COUNT; i++)
    {
        //check if the config contains something (by camFileName)
        if (settings.cameraConfigs[i].camFileName.length() > 0)
        {
            Camera *tempCamPtr = NULL;
            tempCamPtr = startCamFromFile(settings.cameraConfigs[i].camFileName, _outQCount);
            if (tempCamPtr)
            {
                cams.push_back(tempCamPtr);
            }
        }
    }
    ROS_INFO("Cameras initiallization loaded %d cameras", (int)cams.size());
};

//TODO: update configs in runtime
bool updateCams(vector<Camera *> &cams)
{
    for (auto i : cams)
    {
        i->loadAllConfig();
    }
}

void storeCams(vector<Camera *> &cams)
{
    //create the CamConfigs folder if there is not one
    struct stat buffer;
    if (stat("CamConfigs/", &buffer) == -1)
    {
        mkdir("CamConfigs/", ACCESSPERMS);
    }
    for (auto i : cams)
    {
        i->storeAllConfig();
    }
}

bool Camera::loadAllConfig()
{
    FileStorage fs("CamConfigs/" + config_filename, cv::FileStorage::READ);
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
    FileStorage fsr("CamConfigs/" + config_filename, cv::FileStorage::READ);
    int driverType = 0;
    fsr["driverType"] >> driverType;
    fsr.release();

    FileStorage fs("CamConfigs/" + config_filename, cv::FileStorage::WRITE);
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
    fsHelper::readOrDefault(node["haveROI"], haveROI, false);
    fsHelper::readOrDefault(node["capture_width"], capture_width, 0);
    fsHelper::readOrDefault(node["capture_height"], capture_height, 0);
    fsHelper::readOrDefault(node["cameraMatrix"], cameraMatrix);
    fsHelper::readOrDefault(node["distCoeffs"], distCoeffs);
    fsHelper::readOrDefault(node["rotationVec"], rotationVec, {0, 0, 0});
    fsHelper::readOrDefault(node["translationVec"], translationVec, {0, 0, 0});
    fsHelper::readOrDefault(node["minReadDelay_ms"], minReadDelay_ms, 12);
    fsHelper::readOrDefault(node["maxReadDelay_ms"], maxReadDelay_ms, 100);

    //calculate the valur of rotaiton matrix and its inverse
    cv::Rodrigues(rotationVec, rotationMat);
    invert(rotationMat, inverseRotationMat);
};

bool Camera::storeBaseParameters(FileStorage &fs)
{
    fs << "CameraBase"
       << "{"
       << "haveROI" << haveROI
       << "capture_width" << capture_width
       << "capture_height" << capture_height
       << "cameraMatrix" << cameraMatrix
       << "distCoeffs" << distCoeffs
       << "rotationVec" << rotationVec
       << "rotationMat" << rotationMat
       << "inverseRotationMat" << inverseRotationMat
       << "translationVec" << translationVec
       << "minReadDelay_ms" << minReadDelay_ms
       << "maxReadDelay_ms" << maxReadDelay_ms
       << "}";
};
