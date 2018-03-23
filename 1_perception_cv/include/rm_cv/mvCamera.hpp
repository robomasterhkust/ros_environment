#include "Camera.hpp"
#include "CameraApi.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace cv;

class mvCamera : public Camera
{
public:
  //
  //inherited
  //
  mvCamera(int outCount);
  mvCamera(int outCount, const FileStorage &fs);
  ~mvCamera();

  bool initialize();
  FrameInfo *getFrame();

  bool loadDriverParameters(const FileStorage &fs);
  bool storeDriverParameters(FileStorage &fs);

  bool getVideoSize(int &width, int &height);
  bool setExposureTime(bool auto_exp, int t);
  //use only if the driver take resources or need restarting when settings changes, otherwise just return true
  bool startStream();
  bool closeStream();
  void info();

  //
  //device specific
  //
  bool savePreset(const int &settingGroup);
  bool readPreset(const int &settingGroup); //int team range : 0,1,2,3  bool initialize();
  void showSettigsPage();

private:
  //buffers
  BYTE *pbyBuffer;
  unsigned char *g_pRgbBuffer;
  int iCameraCounts = 1;
  int iStatus = -1;
  tSdkCameraDevInfo tCameraEnumList;
  int hCamera;
  tSdkCameraCapbility tCapability;
  tSdkFrameHead sFrameInfo;
  int iDisplayFrames = 10000;
  IplImage *iplImage = NULL;
  int channel = 3;

  bool auto_exp;
  double exposure_time;
  int analogGain;
  int iGamma;
  int iContrast;
  int iSaturation;
  int iFrameSpeed;

  timeval camSetTime;
};
