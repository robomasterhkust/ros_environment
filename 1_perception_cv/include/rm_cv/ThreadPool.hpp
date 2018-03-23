#include "ConcurrentQueue.hpp"
#include "Settings.hpp"
#include "Camera.hpp"
#include "V4LCamDriver.hpp"
#include "ArmorDetection.hpp"
#include "StopWatch.hpp"
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std;
using namespace cv;

class ThreadPool
{
  public:
    ThreadPool();
    ~ThreadPool();

    void initialize();

    void doWork();

    void worker();

    //this should only be ran in the main thread
    void doBossWork();

    bool startThreads(const int &count);

    void stopThreads();

    StopWatch stopWatch = StopWatch("ad");
    vector<Camera *> cams;
    LightFinder *lfPtr;
    ArmorProcessor *apPtr;
    vector<thread *> workers;

    atomic<bool> run;
};