#pragma once
#include "Camera.hpp"
#include "opencv2/core/core.hpp"
#include <string>

class VideoIn : public Camera
{
  public:
    VideoIn(int outCount);
    ~VideoIn();
    bool loadDriverParameters(const FileStorage &fs);
    bool storeDriverParameters(FileStorage &fs);

    bool initialize();
    void discardFrame();
    FrameInfo *getFrame();

    void restartCapture();
    bool getVideoSize(int &width, int &height);

    bool setExposureTime(bool auto_exp, int t);
    bool setFormat(int width, int height, bool mjpg = 1);
    bool setBufferSize(int bsize);
    bool setVideoFPS(int fps);

    bool startStream();
    bool closeStream();
    void info();

  private:
    string video_path;
    cv::VideoCapture vid;
    int fps;
    timeval frameTime;
};
