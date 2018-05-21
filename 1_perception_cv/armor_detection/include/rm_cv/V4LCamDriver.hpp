#pragma once
#include "Camera.hpp"
#include "opencv2/core/core.hpp"
#include <string>

class V4LCamDriver : public Camera
{
public:
  V4LCamDriver(int outCount, const string &config_filename);
  ~V4LCamDriver();

  bool loadDriverParameters(const FileStorage &fs);
  bool storeDriverParameters(FileStorage &fs);

  bool writeCamConfig();
  bool readCamConfig();

  bool initialize();
  void discardFrame();
  FrameInfo *getFrame();

  void restartCapture();
  bool getVideoSize(int &width, int &height);
  void info();

  bool setExposureTime(bool auto_exp, int t);
  bool setFormat(int width, int height, bool mjpg = 1);
  bool setBufferSize(int bsize);
  bool setVideoFPS(int fps);

  bool startStream();
  bool closeStream();

private:
  struct MapBuffer
  {
    void *ptr;
    unsigned int size;
  };

  void cvtRaw2Mat(const void *data, cv::Mat &image);
  bool refreshVideoFormat();
  bool initMMap();
  int xioctl(int fd, int request, void *arg);

  int fd; //store the output of open(...)

  MapBuffer *mb;
  int buffer_size;
  int buffr_idx;

  int capture_width;
  int capture_height;
  int format;
  bool auto_exp;
  int exposureTime;

  int cur_frame;
  string video_path;
};