#pragma once
#include <opencv2/opencv.hpp>
#include "linux/videodev2.h"
#include "Settings.hpp"
#include "ConcurrentQueue.hpp"
#include <time.h>
#include <string>
#include <mutex>

//a class for hanfling all sort of camera things,
//one class for one camera/ one set of stereo camera
//each class should implement the reading and storing of parameters
//

using namespace std;
using namespace cv;

class Camera;

class FrameInfo
{
public:
  FrameInfo(const Camera *sourceCamPtr) : sourceCamPtr(sourceCamPtr){};
  FrameInfo(const FrameInfo &);
  //BGR format image
  Mat img;
  //from system's uptime in second and microsecond(0.000001s)
  // timespec uptime;
  // clock_gettime(CLOCK_MONOTONIC, &uptime);
  timeval capTime;
  //Captured coordinate relative to robot's origin/main camera
  Vec3f rotationVec;
  Vec3f translationVec;
  const Camera *sourceCamPtr;
};

class Camera
{
public:
  virtual ~Camera()
  {
    delete[] outQ;
  };

  bool tryRead(); //this function is probably blocking, limited by camera's fps

  void rectifyCoor(cv::Vec3d &crude_Coordinate) const;

  const Mat &getCameraMatrix() const;
  const Mat &getDistCoeffs() const;

  ConcurrentQueue<FrameInfo> *outQ;

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

protected:
  bool loadAllConfig();
  bool storeAllConfig();
  bool loadBaseParameters(const FileStorage &fs);
  bool storeBaseParameters(FileStorage &fs);

  /**
   * @brief 
   *  really try to set the camera to the specified config 
   */
  virtual bool writeCamConfig() = 0;

  /**
   * @brief 
   * read camera's current config 
   */
  virtual bool readCamConfig() = 0;

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
   * @brief Get the Video Size, was dewsigned to do the job but the mat object saved in the queue seems also provided that info
   */
  virtual bool getVideoSize(int &width, int &height) = 0;

  /**
   * @brief 
   * this just provide a way to read and discard a frame from the camera, mostly usless
   * 
   */
  virtual void discardFrame() = 0;

  /**
   * @brief Blocking function call to decode and copy the image from ther camera to the pointed concurrentQueue 
   * 
   * @return FrameInfo* 
   */
  virtual FrameInfo *getFrame() = 0;

  virtual void info() = 0; //print information about the cam

  Camera(int _outQCount, const string &config_filename)
      : outQCount(_outQCount),
        config_filename(config_filename)
  {
    outQ = new ConcurrentQueue<FrameInfo>[outQCount];
    clock_gettime(CLOCK_MONOTONIC, &lastRead);
  };

  const string config_filename;

  unsigned int outQCount;
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
  Mat rotationMat;
  Mat inverseRotationMat;
  Vec3f rotationVec;
  Vec3f translationVec;

  //thread things
  ConcurrentQueue<FrameInfo> *inputQ;
  mutex lockcam;

  //try read one camera
  friend void startCams(const Settings &settings, vector<Camera *> &cams, int _outQCount);
  friend void tryReadCam(const Settings &settings, vector<Camera *> &cams);
  friend bool updateCams(vector<Camera *> &cams);
  friend void storeCams(vector<Camera *> &cams);
  friend Camera *startCamFromFile(const string &filename, int _outQCount);
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
void startCams(const Settings &settings, vector<Camera *> &cams, int _outQCount);

/**
 * @brief Try to update the camera's settings from the config file
 * 
 * @param cams 
 * @return true 
 * @return false 
 */
bool updateCams(vector<Camera *> &cams);
void storeCams(vector<Camera *> &cams);

Camera *startCamFromFile(const string &filename, int _outQCount);
