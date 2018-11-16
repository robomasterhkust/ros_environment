/**
 * @file CamBase.hpp
 * @author Alex Au (alex_acw@outlook.com)
 * @brief a parent class for driving CamBases defined from config files
 * @version 0.1
 * @date 2018-10-06
 * 
 * @copyright Copyright (c) 2018
 * 
 */
#pragma once
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <opencv2/opencv.hpp>
#include <time.h>
#include <string>
#include <mutex>
#include "defines.hpp"

using namespace std;
using namespace cv;

bool fileExist(const string &filename);

class CamBase;

/**
 * @brief the class for storing a frame captured by a CamBase together with the environment variables
 * only shallow copy is implemented for this class for performance reasons
 * deleteing a instance of this will not delete the image, the image is deleted automatically by opencv with shared pointers
 */
class FrameInfo
{
public:
  FrameInfo(const CamBase *sourceCamPtr) : sourceCamPtr(sourceCamPtr){};
  ~FrameInfo();
  void deepCopyTo(FrameInfo &dst);

  //BGR format image
  Mat img;
  std_msgs::Header rosheader;

  const CamBase *sourceCamPtr;
};

class CamBase
{
public:
  virtual ~CamBase(){};

  string getName() const;

  void rectifyCoor(cv::Vec3d &crude_Coordinate, cv::Matx33d &cov) const;

  const Mat &getCameraMatrix() const;
  const Mat &getDistCoeffs() const;

  /**
 * @brief Get the uniqu ID specifying the driver operating the camera object 
 */
  virtual int getDriverID() const = 0;

  virtual bool initialize() = 0; //function to initiallize to prepare for startStream

  /**
   * @brief 
   * stop stream-> loadconfigs -> set CamBase-> read from CamBase-> store configs-> startstream 
   */
  bool setnGetConfig();

  /**
   * @brief 
   *  these functions are used to start and stop CamBase from sending images,
   *  and are supposed to allow the CamBase to update their settings 
   * @return true 
   * @return false 
   */
  virtual bool startStream() = 0;
  virtual bool closeStream() = 0;

  /**
   * @brief solvePNP with own intrinsic matrix and distortion, output is relative to CamBase's optical center
   */
  virtual bool solvePNP(cv::InputArray objectPoints, const vector<Point2f> &imagePoints,
                        cv::OutputArray rvec, cv::OutputArray tvec,
                        bool useExtrinsicGuess = false, int flags = cv::SOLVEPNP_ITERATIVE) const;

  bool loadAllConfig();
  bool storeAllConfig();
  bool loadBaseParameters(const FileStorage &fs);
  bool storeBaseParameters(FileStorage &fs);

  /**
   * @brief configure CamBase to the specified configuration
   */
  virtual bool setCamConfig() = 0;

  /**
   * @brief get CamBase's current config 
   */
  virtual bool getCamConfig() = 0;

  /**
   * @brief loadDriverParameters load CamBase specific configurations from the file, return true when success, doesn't really update the CamBase
   */
  virtual bool loadDriverParameters(const FileStorage &fs) = 0;

  /**
   * @brief 
   * store configs to the file, return true when success, doesn't really read from the CamBase
   */
  virtual bool storeDriverParameters(FileStorage &fs) = 0;

  /**
   * @brief 
   * this just provide a way to read and discard a frame from the CamBase, mostly usless
   * 
   */
  virtual void discardFrame() = 0;

  /**
   * @brief Blocking function call to decode and making a copy of next incoming image 
   * 
   * @return FrameInfo* pointer to the frame object, return NULL if frame is not available 
   */
  virtual FrameInfo *getFrame() = 0;

  virtual void info() = 0; //print information about the cam

protected:
  CamBase(const string &config_filename)
      : config_filename(config_filename)
  {
    clock_gettime(CLOCK_MONOTONIC, &lastRead);
  };

  const string config_filename;

  int minFPS;
  int maxFPS;
  int minReadDelay_ms;
  int maxReadDelay_ms;
  timespec lastRead;

  //CamBase parameters
  Mat cameraMatrix = cv::Mat(Mat::eye(3, 3, CV_32F));
  Mat distCoeffs = cv::Mat(cv::Mat::zeros(5, 1, CV_32F));

  //coordinate relative to robot's origin/main CamBase
  Matx33d rotationMat;
  Matx33d inverseRotationMat;

  //relative geometry of the CamBase to the frame's origin
  //translation relative to origin
  //rotation is the angle-axis representation of the CamBase relative to the heading of the frame
  Vec3d rotationVec;
  Vec3d translationVec;

  mutex lockcam;

  int failCount = 0;

  //try read one CamBase
  friend CamBase *startCamFromFile(const string &filename);
};

/**
  * @brief 
  *     try to read from one CamBase, according to the actual reading rate
  *     it will not try to read CamBase faster than the maximum rate specified
  *     while if some CamBase's minimum rate is reached, this function will try to read it first
  *  
  * @param settings 
  * @param cams 
 */
void tryReadCam(vector<CamBase *> &cams);

CamBase *startCamFromFile(const string &filename);
