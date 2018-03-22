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

  ConcurrentQueue<FrameInfo> *outQ;

  bool loadBaseParameters(const FileStorage &fs);
  bool storeBaseParameters(FileStorage &fs);
  virtual bool loadDriverParameters(const FileStorage &fs) = 0;
  virtual bool storeDriverParameters(FileStorage &fs) = 0;

  bool tryRead(); //this function is probably blocking, limited by camera's fps

  virtual bool initialize() = 0; //function to initiallize to prepare for startStream

  virtual bool getVideoSize(int &width, int &height) = 0;

  virtual FrameInfo *getFrame() = 0;
  virtual bool setExposureTime(bool auto_exp, int t) = 0;

  virtual bool startStream() = 0; //outputs are supposed to be available afther calling this sucessfully (return true)
  virtual bool closeStream() = 0;

  virtual void info() = 0; //print information about the cam

  Mat getCameraMatrix() const;
  Mat getDistCoeffs() const;

protected:
  Camera(int _outQCount) : outQCount(_outQCount)
  {
    outQ = new ConcurrentQueue<FrameInfo>[outQCount];
    clock_gettime(CLOCK_MONOTONIC, &lastRead);
  };
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
  Vec3f rotationVec;
  Vec3f translationVec;

  //thread things
  ConcurrentQueue<FrameInfo> *inputQ;
  atomic<bool> runBool;
  thread *threadPtr;
  mutex lockcam;

  //try read one camera
  friend void tryReadCam(const Settings &settings, vector<Camera *> &cams);
};

//construct objects for all cameras descripted in settings
void loadCams(const Settings &settings, vector<Camera *> &cams, int _outQCount);
void storeCams(const Settings &settings, vector<Camera *> &cams);

//tryReadCam: get a frame from one camera according to the miniman delay set (max fps) and thier pirority
//cam0 in settings.xml have higher pirority then others, meaning that it is read before others and once its read it will be processed first
void tryReadCam(const Settings &settings, vector<Camera *> &cams);