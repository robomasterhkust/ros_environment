#include "Settings.hpp"
#include "Camera.hpp"
#include "V4LCamDriver.hpp"
#include "mvCamera.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <sys/stat.h>
#include "StopWatch.hpp"
#include <chrono>
#include <ctime>
using namespace std;

FrameInfo::FrameInfo(const FrameInfo &target)
{
    target.img.copyTo(img);
    capTime = target.capTime;
    rotationVec = target.rotationVec;
    translationVec = target.translationVec;
    sourceCamPtr = target.sourceCamPtr;
};

bool Camera::loadBaseParameters(const FileStorage &fs)
{
    const FileNode &node = fs["CameraBase"];
    fsHelper::readOrDefault(node["haveROI"], haveROI, false);
    fsHelper::readOrDefault(node["capture_width"], capture_width, 0);
    fsHelper::readOrDefault(node["capture_height"], capture_height, 0);
    fsHelper::readOrDefault(node["cameraMatrix"], cameraMatrix);
    fsHelper::readOrDefault(node["distCoeffs"], distCoeffs);
    fsHelper::readOrDefault(node["rotationVec"], rotationVec);
    fsHelper::readOrDefault(node["translationVec"], translationVec);
    fsHelper::readOrDefault(node["minReadDelay_ms"], minReadDelay_ms, 0);
    fsHelper::readOrDefault(node["maxReadDelay_ms"], maxReadDelay_ms, 100);
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
       << "}";
};

bool Camera::tryRead()
{
    if (lockcam.try_lock())
    {
        clock_gettime(CLOCK_MONOTONIC, &lastRead);
        FrameInfo *tempout = NULL;
        do
        {
            delete tempout;
            do
            {
                tempout = getFrame();
            } while (!tempout);
        } while (tempout->img.empty());

        outQ[0].enqueue(tempout);
        for (int i = 1; i < outQCount; i++)
        {
            outQ[i].enqueue(new FrameInfo(*tempout));
        }
        lockcam.unlock();
        return true;
    }
    return false;
};

Mat Camera::getCameraMatrix() const
{
    return cameraMatrix;
};
Mat Camera::getDistCoeffs() const
{
    return distCoeffs;
};

void loadCams(const Settings &settings, vector<Camera *> &cams, int _outQCount)
{
    struct stat buffer;
    if (stat("CamConfigs/", &buffer) == -1)
    {
        mkdir("CamConfigs/", 0700);
    }
    cams.clear();
    for (int i = 0; i < MAX_CAM_COUNT; i++)
    {
        //check if the config contains something (by camFileName)
        if (settings.cameraConfigs[i].camFileName.length() > 0)
        {
            //create device specific object
            Camera *tempPtr;
            switch (settings.cameraConfigs[i].camDriver)
            {
            case 0: //0 is for V4l Drivers
                tempPtr = new V4LCamDriver(_outQCount);
                cams.push_back(tempPtr);
                break;

            case 1: //1 is for mvux cameras
                tempPtr = new mvCamera(_outQCount);
                cams.push_back(tempPtr);
                break;
            }

            //read device configurations
            if (fileExist("CamConfigs/" + settings.cameraConfigs[i].camFileName))
            {
                FileStorage fs("CamConfigs/" + settings.cameraConfigs[i].camFileName, cv::FileStorage::READ);
                cams.back()->loadBaseParameters(fs);
                cams.back()->loadDriverParameters(fs);
                fs.release();
            }
            //overwrite file or create a default one
            FileStorage fs("CamConfigs/" + settings.cameraConfigs[i].camFileName, cv::FileStorage::WRITE);
            cams.back()->storeBaseParameters(fs);
            cams.back()->storeDriverParameters(fs);
            fs.release();
        }
    }
};

void storeCams(const Settings &settings, vector<Camera *> &cams)
{
    struct stat buffer;
    if (stat("CamConfigs/", &buffer) == -1)
    {
        mkdir("CamConfigs/", 0700);
    }
    cams.clear();
    int j = 0;
    for (int i = 0; i < MAX_CAM_COUNT; i++)
    {
        //check if the config contains something
        if (settings.cameraConfigs[i].camFileName.length() > 0)
        {
            FileStorage fs("CamConfigs/" + settings.cameraConfigs[i].camFileName, cv::FileStorage::WRITE);
            cams[j]->storeBaseParameters(fs);
            cams[j]->storeDriverParameters(fs);
            fs.release();
            j++;
        }
    }
};

void tryReadCam(const Settings &settings, vector<Camera *> &cams)
{
    //TODO: debug?
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