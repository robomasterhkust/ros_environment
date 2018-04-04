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
#include "ThreadPool.hpp"

using namespace std;
using namespace cv;

extern Settings settings;

ThreadPool::ThreadPool() : run(false){};

ThreadPool::~ThreadPool()
{
    stopThreads();
    for (int i = 0; i < cams.size(); i++)
    {
        delete cams[i];
    }
    delete lfPtr;
    delete apPtr;
};

void ThreadPool::initialize()
{
    loadCams(settings, cams, 2);
    for (auto i : cams)
    {
        i->initialize();
        i->startStream();
    }

    //setup the concurrent queues used to join pipelines
    //the setup is now hardcoded for single camera cam[0]
    //TODO: automatic object creation and data flow handling according to cameras specified
    lfPtr = new LightFinder(&cams[0]->outQ[0], 2, 2);
    apPtr = new ArmorProcessor(lfPtr->outputQArr[0], 1, 2);
};

void ThreadPool::doWork()
{
    //TODO:
    if (!apPtr->tryWork())
        if (!lfPtr->tryWork())
            tryReadCam(settings, cams);
    //cams[0]->tryRead();
};

void ThreadPool::worker()
{
    while (run)
    {
        doWork();
    }
};

/**
 * @brief work that the main thread should do
 */
void ThreadPool::doBossWork()
{
    //TODO: Handle ROS message processing
    static FrameInfo *tempOut;
    static LightStorage *tempLS;
    static ArmorStorage *tempAS;
    if (cams[0]->outQ[1].dequeue(tempOut))
    {
        imshow("frame", tempOut->img);
        delete tempOut;
    }
    if (lfPtr->outputQArr[1]->dequeue(tempLS))
    {
        tempLS->draw();
        delete tempLS;
    }
    if (apPtr->outputQArr[0]->dequeue(tempAS))
    {
        tempAS->draw();
        delete tempAS;
        stopWatch.lap();
    }
};

bool ThreadPool::startThreads(const int &count)
{
    if (run)
    {
        return false;
    }
    else
    {
        run = true;
        for (int i = 0; i < count; i++)
        {
            cout << "created thread " << i << endl;
            workers.push_back(new thread(&ThreadPool::worker, this));
        }
    }
};

void ThreadPool::stopThreads()
{
    run = false;
    while (workers.size() != 0)
    {
        workers.back()->join();
        delete workers.back();
        workers.pop_back();
    }
};