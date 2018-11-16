#include "defines.hpp"
#include "Settings.hpp"
#include "Camera.hpp"
#include "StopWatch.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <sys/stat.h>
#include <chrono>
#include <ctime>
#include <mutex>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "ROSCamIn.hpp"
#include "cvThreadPool.hpp"
#include "main.hpp"
#include <stdbool.h>

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

void tryReadCam(const Settings &settings, vector<Camera *> &cams)
{
    timespec uptime;
    clock_gettime(CLOCK_MONOTONIC, &uptime);
    int tempReadDelay[cams.size()];
    for (int i = 0; i < cams.size(); i++)
        if (cams[i])
        {
            tempReadDelay[i] = (uptime.tv_sec - cams[i]->lastRead.tv_sec) * 1000 +
                               (uptime.tv_nsec - cams[i]->lastRead.tv_nsec) / 1000000;
            if (tempReadDelay[i] > cams[i]->minReadDelay_ms)
            {
                bool readSucess = cams[i]->tryRead();
                if (!readSucess && (tempReadDelay[i] > cams[i]->maxReadDelay_ms))
                {
                    if (cams[i]->lockcam.try_lock())
                    {

                        //camera failed, try to restart here
                        ROS_WARN("camera from %s timout, restarting", cams[i]->config_filename.c_str());
                        cams[i]->initialize();
                        cams[i]->applySetting();
                        cams[i]->startStream();
                        cams[i]->lockcam.unlock();
                    }
                }
                return;
            }
        }
};

/**
 * @brief initiallize a camera object form the specified file name, and check if the camera is accessable
 * 
 * @param filename 
 * @return Camera* NULL if the camera is not available, otherwise point to the camera object
 */
Camera *startCamFromFile(const string &filename)
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
        case ROS_IMAGE_IN:
            ROS_INFO("creating ROS camera reader");
            tempCamera = new ROSCamIn(filename);
            break;
        }
        fs.release();

        if (tempCamera)
        {
            tempCamera->loadAllConfig();
            if (tempCamera->initialize())
            {
                tempCamera->info();
                if (tempCamera->applySetting())
                {
                    ROS_INFO("Sucessfully apply setting to camera object from %s", filename.c_str());
                }
                else
                {
                    ROS_INFO("Applying setting to camera object from %s failed, start camera anyway", filename.c_str());
                }

                if (tempCamera->startStream())
                {
                    ROS_INFO("Sucessfully created camera object from %s", filename.c_str());
                    tempCamera->showSettingWindow();
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
        FileStorage fs("CamConfigs/" + filename, cv::FileStorage::WRITE);
        fs << "driverType" << -1;
        mode_t mode = 00777;
        string temp = "CamConfigs/" + filename;
        chmod(temp.c_str(), mode);

        ROS_INFO("Cam Config file CamConfigs\\%s not found\n", filename.c_str());
    }
    return NULL;
}

/**
 * @brief Create a Cams object
 * 
 * @param settings the data read from the setting file
 * @param cams the vector of pointers to camera objects
 */
void startCams(const Settings &settings, vector<Camera *> &cams)
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
            tempCamPtr = startCamFromFile(settings.cameraConfigs[i].camFileName);
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
    fsHelper::readOrDefault(node["minReadDelay_ms"], minReadDelay_ms, 0);
    fsHelper::readOrDefault(node["maxReadDelay_ms"], maxReadDelay_ms, 100);
    fsHelper::readOrDefault(node["lightFilterSetting"], lightFilterSetting);

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
       << "translationVec" << translationVec
       << "minReadDelay_ms" << minReadDelay_ms
       << "maxReadDelay_ms" << maxReadDelay_ms
       << "lightFilterSetting" << lightFilterSetting
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
    createTrackbar("use HSV", this->config_filename + " control panel", &this->lightFilterSetting.UseHSV, 1);

    createTrackbar("blue Min H", this->config_filename + " control panel", &this->lightFilterSetting.HSVMinBlue[0], 255);
    createTrackbar("blue Max H", this->config_filename + " control panel", &this->lightFilterSetting.HSVMaxBlue[0], 255);
    createTrackbar("blue Min S", this->config_filename + " control panel", &this->lightFilterSetting.HSVMinBlue[1], 255);
    createTrackbar("blue Max S", this->config_filename + " control panel", &this->lightFilterSetting.HSVMaxBlue[1], 255);
    createTrackbar("blue Min V", this->config_filename + " control panel", &this->lightFilterSetting.HSVMinBlue[2], 255);
    createTrackbar("blue Max V", this->config_filename + " control panel", &this->lightFilterSetting.HSVMaxBlue[2], 255);
    createTrackbar("Red Min H", this->config_filename + " control panel", &this->lightFilterSetting.HSVMinRed[0], 255);
    createTrackbar("Red Max H", this->config_filename + " control panel", &this->lightFilterSetting.HSVMaxRed[0], 255);
    createTrackbar("Red Min S", this->config_filename + " control panel", &this->lightFilterSetting.HSVMinRed[1], 255);
    createTrackbar("Red Max S", this->config_filename + " control panel", &this->lightFilterSetting.HSVMaxRed[1], 255);
    createTrackbar("Red Min V", this->config_filename + " control panel", &this->lightFilterSetting.HSVMinRed[2], 255);
    createTrackbar("Red Max V", this->config_filename + " control panel", &this->lightFilterSetting.HSVMaxRed[2], 255);

    createTrackbar("blue Min B", this->config_filename + " control panel", &this->lightFilterSetting.BGRMinBlue[0], 255);
    createTrackbar("blue Max B", this->config_filename + " control panel", &this->lightFilterSetting.BGRMaxBlue[0], 255);
    createTrackbar("blue Min G", this->config_filename + " control panel", &this->lightFilterSetting.BGRMinBlue[1], 255);
    createTrackbar("blue Max G", this->config_filename + " control panel", &this->lightFilterSetting.BGRMaxBlue[1], 255);
    createTrackbar("blue Min R", this->config_filename + " control panel", &this->lightFilterSetting.BGRMinBlue[2], 255);
    createTrackbar("blue Max R", this->config_filename + " control panel", &this->lightFilterSetting.BGRMaxBlue[2], 255);
    createTrackbar("Red Min B", this->config_filename + " control panel", &this->lightFilterSetting.BGRMinRed[0], 255);
    createTrackbar("Red Max B", this->config_filename + " control panel", &this->lightFilterSetting.BGRMaxRed[0], 255);
    createTrackbar("Red Min G", this->config_filename + " control panel", &this->lightFilterSetting.BGRMinRed[1], 255);
    createTrackbar("Red Max G", this->config_filename + " control panel", &this->lightFilterSetting.BGRMaxRed[1], 255);
    createTrackbar("Red Min R", this->config_filename + " control panel", &this->lightFilterSetting.BGRMinRed[2], 255);
    createTrackbar("Red Max R", this->config_filename + " control panel", &this->lightFilterSetting.BGRMaxRed[2], 255);
};
