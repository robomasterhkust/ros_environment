#pragma once
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <opencv2/opencv.hpp>
#include "linux/videodev2.h"
#include "Settings.hpp"
#include "ConcurrentQueue.hpp"
#include <time.h>
#include <string>
#include <mutex>

//a class for handling all sort of camera things,
//one class for one camera/ one set of stereo camera
//each class should implement the reading and storing of parameters
//

using namespace std;
using namespace cv;

class Camera;

//only shallow copy is implemented for this class for performance reasons
/**
 * @brief the class for storing a frame captured by a camera together with the environment variables
 * only shallow copy is implemented for this class for performance reasons
 * deleteing a instance of this will not delete the image, the image is deleted automatically by opencv with shared pointers
 */
class FrameInfo
{
public:
  FrameInfo(const Camera *sourceCamPtr) : sourceCamPtr(sourceCamPtr){};
  ~FrameInfo();
  void copyTo(FrameInfo &dst);

  //BGR format image
  Mat img;
  std_msgs::Header rosheader;

  //Captured coordinate relative to robot's origin/main camera, coordinate frame specified in the header
  Vec3f rotationVec;
  Vec3f translationVec;
  const Camera *sourceCamPtr;
};

class Camera
{
public:
  virtual ~Camera(){};

  string getName() const;

  void showSettingWindow();

  bool tryRead(); //this function is probably blocking, limited by camera's fps

  void rectifyCoor(cv::Vec3d &crude_Coordinate, cv::Matx33d &cov) const;

  const Mat &getCameraMatrix() const;
  const Mat &getDistCoeffs() const;

  ConcurrentQueue<FrameInfo> outQ;

  virtual bool initialize() = 0; //function to initiallize to prepare for startStream

  /**
   * @brief 
   * stop stream-> loadconfigs -> set camera-> read from camera-> store configs-> startstream 
   */
  bool applySetting();

  /**
   * @brief 
   *  these functions are used to start and stop camera from sending images,
   *  and are supposed to allow the camera to update their settings 
   * @return true 
   * @return false 
   */
  virtual bool startStream() = 0;
  virtual bool closeStream() = 0;

  virtual bool solvePNP(cv::InputArray objectPoints, const vector<Point2f> &imagePoints,
                        cv::OutputArray rvec, cv::OutputArray tvec,
                        bool useExtrinsicGuess = false, int flags = cv::SOLVEPNP_ITERATIVE) const;

  LightFilterSetting lightFilterSetting;

protected:
  bool loadAllConfig();
  bool storeAllConfig();
  bool loadBaseParameters(const FileStorage &fs);
  bool storeBaseParameters(FileStorage &fs);

  /**
   * @brief 
   *  really try to set the camera to the specified config 
   */
  virtual bool setCamConfig() = 0;

  /**
   * @brief 
   * get camera's current config 
   */
  virtual bool getCamConfig() = 0;

  /**
   * @brief loadDriverParameters
   * load configs from the file, return true when success, doesn't really update the camera
   */
  virtual bool loadDriverParameters(const FileStorage &fs) = 0;

  /**
   * @brief 
   * store configs to the file, return true when success, doesn't really read from the camera
   */
  virtual bool storeDriverParameters(FileStorage &fs) = 0;

  /**
   * @brief 
   * this just provide a way to read and discard a frame from the camera, mostly usless
   * 
   */
  virtual void discardFrame() = 0;

  /**
   * @brief Blocking function call to decode and copy the image from ther camera to the pointed concurrentQueue 
   * 
   * @return FrameInfo* if frame is not available, NULL will be returned
   */
  virtual FrameInfo *getFrame() = 0;

  virtual void info() = 0; //print information about the cam

  Camera(const string &config_filename)
      : config_filename(config_filename),
        outQ(4)
  {
    clock_gettime(CLOCK_MONOTONIC, &lastRead);
  };

  const string config_filename;

  bool haveROI;
  int capture_width;
  int capture_height;
  int minReadDelay_ms;
  int maxReadDelay_ms;
  timespec lastRead;

  //camera parameters
  Mat cameraMatrix;
  Mat distCoeffs;

  //coordinate relative to robot's origin/main camera
  Matx33d rotationMat;
  Matx33d inverseRotationMat;

  //relative geometry of the camera to the frame's origin
  //translation relative to origin
  //rotation is the angle-axis representation of the camera relative to the heading of the frame
  Vec3d rotationVec;
  Vec3d translationVec;

  //thread things
  mutex lockcam;
  //ConcurrentQueue<FrameInfo> *inputQ;

  //try read one camera
  friend void startCams(const Settings &settings, vector<Camera *> &cams);
  friend void tryReadCam(const Settings &settings, vector<Camera *> &cams);
  friend bool updateCams(vector<Camera *> &cams);
  friend void storeCams(vector<Camera *> &cams);
  friend Camera *startCamFromFile(const string &filename);

private:
  int failCount = 0;
};

/**
  * @brief 
  *     try to read from one camera, according to the actual reading rate
  *     it will not try to read camera faster than the maximum rate specified
  *     while if some camera's minimum rate is reached, this function will try to read it first
  *  
  * @param settings 
  * @param cams 
 */
void tryReadCam(const Settings &settings, vector<Camera *> &cams);

//construct objects for all cameras descripted in settings
void startCams(const Settings &settings, vector<Camera *> &cams);

/**
 * @brief Try to update the camera's settings from the config file
 * 
 * @param cams 
 * @return true 
 * @return false 
 */
bool updateCams(vector<Camera *> &cams);
void storeCams(vector<Camera *> &cams);

Camera *startCamFromFile(const string &filename);
