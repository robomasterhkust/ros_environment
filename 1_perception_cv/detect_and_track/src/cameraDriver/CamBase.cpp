#include "defines.hpp"
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
#include <sys/stat.h>
#include "helpers.hpp"
#include "V4LCamDriver.hpp"
#include "FlycapCam.hpp"
#include "mvCamera.hpp"

using namespace std;

bool fileExist(const string &filename)
{
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

void FrameInfo::deepCopyTo(FrameInfo &dst)
{
    //perform deep copy
    this->img.copyTo(dst.img);
    dst.rosheader = this->rosheader;
    dst.sourceCamPtr = this->sourceCamPtr;
};

FrameInfo::~FrameInfo(){};

string CamBase::getName() const
{
    return config_filename;
};

void CamBase::rectifyCoor(cv::Vec3d &crude_Coordinate, cv::Matx33d &cov) const
{
    ROS_INFO("CamBase::rectifyCoor : converting");
    ROS_INFO("CamBase coor (%f,%f,%f)", crude_Coordinate[0], crude_Coordinate[1], crude_Coordinate[2]);

    cov = inverseRotationMat * cov * inverseRotationMat.t();

    crude_Coordinate = inverseRotationMat * crude_Coordinate - translationVec;
    //ROS_INFO("into gimbal coor (%f,%f,%f)", crude_Coordinate[0], crude_Coordinate[1], crude_Coordinate[2]);
};

const Mat &CamBase::getCameraMatrix() const
{
    return cameraMatrix;
};

const Mat &CamBase::getDistCoeffs() const
{
    return distCoeffs;
};

bool CamBase::setnGetConfig()
{
    bool success = true;
    success &= this->setCamConfig();
    success &= this->getCamConfig();
    success &= this->storeAllConfig();
    return success;
};

bool CamBase::loadAllConfig()
{
    FileStorage fs(CAM_CONFIG_FOLDER_STR + config_filename, cv::FileStorage::READ);
    if (fs.isOpened())
    {
        bool temp1 = loadBaseParameters(fs);
        bool temp2 = loadDriverParameters(fs);

        if (temp1 && temp2)
        {
            fs.release();
            ROS_INFO("CamBase load from %s success", config_filename.c_str());
            return true;
        }
    }
    else
    {
        ROS_INFO("CamBase load from %s failed", config_filename.c_str());
    }

    fs.release();
    return false;
};

bool CamBase::storeAllConfig()
{

    FileStorage fs(CAM_CONFIG_FOLDER_STR + config_filename, cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "driverType" << getDriverID();
        storeBaseParameters(fs);
        storeDriverParameters(fs);
        fs.release();
        ROS_INFO("CamBase store to %s success, driver type %d", config_filename.c_str(), getDriverID());
        return true;
    }
    else
    {
        fs.release();
        ROS_INFO("CamBase store to %s failed", config_filename.c_str());
        return false;
    }
};

bool CamBase::loadBaseParameters(const FileStorage &fs)
{
    cv::Vec3f tempRV;
    const FileNode &node = fs["CamBaseBase"];
    fsHelper::readOrDefault(node["cameraMatrix"], cameraMatrix, cv::Mat(Mat::eye(3, 3, CV_32F)));
    Mat temp;
    fsHelper::readOrDefault(node["distCoeffs"], temp);
    if (temp.cols == 1 && (temp.rows == 4 || temp.rows == 5 || temp.rows == 8))
    {
        distCoeffs = temp;
    }
    else
    {
        distCoeffs = cv::Mat(Mat::zeros(4, 1, CV_32F));
    }

    fsHelper::readOrDefault(node["rotationVec"], rotationVec, {0, 0, 0});
    fsHelper::readOrDefault(node["translationVec"], translationVec, {0, 0, 0});
    fsHelper::readOrDefault(node["minReadDelay_ms"], minReadDelay_ms, 0);
    fsHelper::readOrDefault(node["maxReadDelay_ms"], maxReadDelay_ms, 100);

    //check camera matrix
    //calculate the value of rotaiton matrix and its inverse
    cv::Rodrigues(rotationVec, rotationMat);
    invert(rotationMat, inverseRotationMat);
};

bool CamBase::storeBaseParameters(FileStorage &fs)
{
    fs << "CamBaseBase"
       << "{"
       << "cameraMatrix" << cameraMatrix
       << "distCoeffs" << distCoeffs
       << "rotationVec" << rotationVec
       << "translationVec" << translationVec
       << "minReadDelay_ms" << minReadDelay_ms
       << "maxReadDelay_ms" << maxReadDelay_ms
       << "}";
};

bool CamBase::solvePNP(cv::InputArray objectPoints, const vector<Point2f> &imagePoints,
                       cv::OutputArray rvec, cv::OutputArray tvec,
                       bool useExtrinsicGuess, int flags) const
{
    return cv::solvePnP(objectPoints, imagePoints, this->cameraMatrix, this->distCoeffs, rvec, tvec, useExtrinsicGuess, flags);
};

/**
 * @brief initiallize a CamBase object form the specified file name, and check if the CamBase is accessable
 * @return NULL if the CamBase is not available, otherwise point to the CamBase object
 */
CamBase *startCamFromFile(const string &filename)
{
    CamBase *tempCamBase = NULL;
    int driverType;
    string path = CAM_CONFIG_FOLDER_STR + filename;
    if (fileExist(path))
    {
        ROS_INFO("Cam Config file %s found", path.c_str());
        FileStorage fs(path, cv::FileStorage::READ);
        fsHelper::readOrDefault(fs["driverType"], driverType, 0);
        switch (driverType)
        {
        case V4LCamera:
            ROS_INFO("creating V4L camera object");
            tempCamBase = new V4LCamDriver(filename);
            break;

#ifdef WITH_MVSDK
        case MVSDKDriver:
            ROS_INFO("creating mvux camera object");
            tempCamBase = new mvCamera(filename);
            break;
#endif

#ifdef WITH_FLYCAP
        case FLYCAP_CAMERA:
            ROS_INFO("creating Flycapture camera reader");
            tempCamBase = new FlycapCam(filename);
            break;
#endif
        }
        fs.release();

        if (tempCamBase)
        {
            if (tempCamBase->loadAllConfig())
            {
                if (tempCamBase->initialize())
                {
                    tempCamBase->info();
                    if (tempCamBase->setnGetConfig())
                    {
                        ROS_INFO("Sucessfully apply setting to CamBase object from %s", filename.c_str());
                    }
                    else
                    {
                        ROS_INFO("Applying setting to CamBase object from %s failed, start CamBase anyway", filename.c_str());
                    }

                    if (tempCamBase->startStream())
                    {
                        ROS_INFO("Sucessfully created CamBase object from %s", filename.c_str());
                        return tempCamBase;
                    }
                    else
                    {
                        ROS_INFO("CamBase object from %s startStream failed", filename.c_str());
                    }
                }
                else
                {
                    ROS_INFO("CamBase object from %s initialize failed", filename.c_str());
                }
            }

            ROS_INFO("CamBase object from %s cannot be started", filename.c_str());
            tempCamBase->storeAllConfig();
        }
        else
        {
            //device unsupported, or config file broken
            ROS_INFO("CamBase specified in %s unsupported, or file is broken", filename.c_str());
        }
    }
    else
    {
        mode_t mode = ACCESSPERMS;
        umask(0);
        int err = mkdir(CAM_CONFIG_FOLDER_STR, mode);
        if (err > 0 && err != EEXIST)
        {
            ROS_WARN("folder %s not found and failed to create: %d",
                     CAM_CONFIG_FOLDER_STR,
                     err);
        }
        FileStorage fs(CAM_CONFIG_FOLDER_STR + filename, cv::FileStorage::WRITE);
        fs << "driverType" << -1;
        string temp = CAM_CONFIG_FOLDER_STR + filename;
        chmod(temp.c_str(), mode);
        fs.release();

        ROS_INFO("Cam Config file \%s not found, template created with driverType -1\n", path.c_str());
    }
    return NULL;
}
