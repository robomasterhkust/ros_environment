#include "Camera.hpp"
#include "VideoIn.hpp"
#include "opencv2/core/core.hpp"
#include <string>
#include <chrono>

VideoIn::VideoIn()
    : Camera()
{
    timespec tempTime;
    clock_gettime(CLOCK_MONOTONIC, &tempTime);
    frameTime.tv_sec = tempTime.tv_sec;
    frameTime.tv_usec = tempTime.tv_nsec / 1000;
};

VideoIn::~VideoIn()
{
    vid.release();
};

bool VideoIn::loadDriverParameters(const FileStorage &fs)
{
    const FileNode &node = fs["VideoIn"];
    fsHelper::readOrDefault(node["video_path"], video_path, (string) "test.avi");
    fsHelper::readOrDefault(node["frequency"], fps, 30);
};
bool VideoIn::storeDriverParameters(FileStorage &fs)
{
    fs << "VideoIn"
       << "{"
       << "video_path" << video_path
       << "frequency" << fps
       << "}";
};

bool VideoIn::initialize()
{
    vid = cv::VideoCapture(video_path);
    if (vid.isOpened())
    {
        cout << video_path << " open success\n";
    }
    else
        cout << video_path << " open failed\n";

    return vid.isOpened();
};

void VideoIn::discardFrame()
{
    frameTime.tv_usec += 1000000 / fps;
    frameTime.tv_sec += frameTime.tv_usec / 1000000;
    frameTime.tv_usec %= 1000000;
    cv::Mat temp;
    vid.read(temp);
};

FrameInfo *VideoIn::getFrame()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps));
    if (vid.isOpened())
    {
        frameTime.tv_usec += 1000000 / fps; 
        frameTime.tv_sec += frameTime.tv_usec / 1000000;
        frameTime.tv_usec %= 1000000;
        FrameInfo *out = new FrameInfo(this);
        out->capTime = frameTime;
        vid.read(out->img);
        return out;
    }
    else
    {
        cout << "video " << video_path << " not available\n";
        return NULL;
    }
};

void VideoIn::restartCapture()
{
    return;
};
bool VideoIn::getVideoSize(int &width, int &height)
{
    cv::Mat temp;
    vid.retrieve(temp);
    width = temp.size().width;
    height = temp.size().height;
};

bool VideoIn::setExposureTime(bool auto_exp, int t) { return true; };
bool VideoIn::setFormat(int width, int height, bool mjpg) { return true; };
bool VideoIn::setBufferSize(int bsize) { return true; };
bool VideoIn::setVideoFPS(int fps)
{
    this->fps = fps;
};

bool VideoIn::startStream() { return true; };
bool VideoIn::closeStream() { return true; };
void VideoIn::info()
{
    cout << "Reading from video file" << video_path << endl;
    return;
};
