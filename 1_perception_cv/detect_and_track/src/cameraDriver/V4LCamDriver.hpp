#pragma once
#include "CamBase.hpp"
#include "opencv2/core/core.hpp"
#include "ros/ros.h"
#include <string>

class V4LCamDriver : public CamBase
{
public:
  V4LCamDriver(const string &config_filename);
  ~V4LCamDriver();

  bool loadDriverParameters(const FileStorage &fs);
  bool storeDriverParameters(FileStorage &fs);
  int getDriverID() const { return V4LCamera; };

  bool setCamConfig();
  bool getCamConfig();

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
  int buffer_size = 3;
  int buffr_idx;

  int capture_width = 1280;
  int capture_height = 720;
  int format = 0;
  bool auto_exp = 0;

  int exposureTime = 30; //in 0.1 ms

  int cur_frame;
  string video_path;

  ros::Duration toEpochOffset;
};