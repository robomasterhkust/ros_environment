#pragma once
#include <flycapture/FlyCapture2.h>
#include <opencv2/opencv.hpp>
#include "CamBase.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

class FlycapCam : public CamBase
{
  public:
    FlycapCam(const string &config_filename);
    ~FlycapCam();

    int getDriverID() const { return FLYCAP_CAMERA; };

    bool initialize();
    void discardFrame();

    /**
   * @brief Get the Frame object
   * this operation is not locked by mutex
   * @return FrameInfo* 
   */
    FrameInfo *getFrame();

    cv_bridge::CvImage *getFrameROS();

    bool loadDriverParameters(const FileStorage &fs);
    bool storeDriverParameters(FileStorage &fs);

    bool setCamConfig();
    bool getCamConfig();

    bool startStream();
    bool closeStream();
    void info();

    bool showCamProperty(const char *name,
                         FlyCapture2::PropertyType type);

    bool setCamProperty(
        const char *name,
        FlyCapture2::PropertyType type,
        bool use_auto,
        bool use_abs,
        float value_abs,
        unsigned int valueA,
        unsigned int valueB);

    bool getCamProperty(
        FlyCapture2::PropertyType type,
        FlyCapture2::Property *prop);

    float getTemperature();

    bool solvePNP(cv::InputArray objectPoints, const vector<Point2f> &imagePoints,
                  cv::OutputArray rvec, cv::OutputArray tvec,
                  bool useExtrinsicGuess, int flags) const override;

    int getSN()
    {
        return serialNumber;
    }

  private:
    //driver objects
    FlyCapture2::BusManager busMgr;
    FlyCapture2::Camera *pCamera; // pointer to the CamBase

    //opencv preprocess
    bool resizeHalf;

    //main CamBase info
    FlyCapture2::CameraInfo camInfo;
    FlyCapture2::PGRGuid guid; // A GUID to the CamBase, uniquely identify a CamBase.

    FlyCapture2::FC2Config CamBaseConfig;

    unsigned int serialNumber = 0; // serial Number unique for each CamBase

    //format7 related
    FlyCapture2::Format7PacketInfo format7PacketInfo;
    FlyCapture2::Format7ImageSettings format7ImageSettings;
    unsigned int packagesize;
    float packagesizepercent;

    FlyCapture2::Property properties[17];
};
