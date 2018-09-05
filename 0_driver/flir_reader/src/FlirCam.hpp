#pragma once
#include <flycapture/FlyCapture2.h>
#include <opencv2/opencv.hpp>
#include "CamBase.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

class PointGreyCamera : public Camera
{
  public:
    //
    //inherited
    //
    PointGreyCamera(const string &config_filename);
    ~PointGreyCamera();

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

  private:
    //driver objects
    FlyCapture2::BusManager busMgr;
    FlyCapture2::Camera *pCamera; // pointer to the Camera

    //opencv preprocess
    bool resizeHalf;

    //main camera info
    FlyCapture2::CameraInfo camInfo;
    FlyCapture2::PGRGuid guid; // A GUID to the camera, uniquely identify a camera.

    FlyCapture2::FC2Config cameraConfig;
    FlyCapture2::CameraInfo cameraInfo;

    unsigned int serialNumber = 0; // serial Number unique for each camera

    //format7 related
    FlyCapture2::Format7PacketInfo format7PacketInfo;
    FlyCapture2::Format7ImageSettings format7ImageSettings;
    unsigned int packagesize;
    float packagesizepercent;

    FlyCapture2::Property properties[17];

    bool brightness_use_abs;
    bool brightness_use_auto;
    float brightness_abs;
    unsigned int brightness;

    bool frameRate_use_abs;
    bool frameRate_use_auto;
    float frameRate_abs;
    unsigned int frameRate;

    bool exposure_use_abs;
    bool exposure_use_auto;
    float exposure_abs;
    unsigned int exposure;

    bool hue_use_abs;
    float hue_abs;
    unsigned int hue;

    bool saturation_use_abs;
    float saturation_abs;
    unsigned int saturation;

    bool sharpness_use_abs;
    float sharpness_abs;
    unsigned int sharpness;

    //white balance must be in absolute mode
    bool WB_use_auto;
    unsigned int WB_red;
    unsigned int WB_blue;

    bool gamma_use_abs;
    bool gamma_use_auto;
    float gamma_abs;
    unsigned int gamma;

    bool shutter_use_abs;
    bool shutter_use_auto;
    float shutter_abs;
    unsigned int shutter;

    bool gain_use_abs;
    bool gain_use_auto;
    float gain_abs;
    unsigned int gain;
};
