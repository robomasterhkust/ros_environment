#include "CamBase.hpp"
#include "mvux_camera/CameraDefine.h"
#include "mvux_camera/CameraStatus.h"
#include "mvux_camera/CameraApi.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "defines.hpp"
using namespace cv;

class mvCamera : public CamBase
{
public:
  mvCamera(const string &config_filename);
  ~mvCamera();
  int getDriverID() const { return MVSDKDriver; };

  bool initialize();
  void discardFrame();
  FrameInfo *getFrame();

  bool loadDriverParameters(const FileStorage &fs);
  bool storeDriverParameters(FileStorage &fs);

  bool setCamConfig();
  bool getCamConfig();

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
  int hCamera;

  //buffers
  BYTE *pbyBuffer;
  unsigned char *g_pRgbBuffer;
  int iCameraCounts = 1;
  int iStatus = -1;
  tSdkCameraDevInfo tCameraEnumList;
  tSdkCameraCapbility tCapability;
  tSdkFrameHead sFrameInfo;
  tSdkImageResolution tImageResolution;
  tSdkFrameSpeed tFrameSpeed;
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
  int frameWidth;
  int frameHeight;

  ros::Time camSetTime;
};
