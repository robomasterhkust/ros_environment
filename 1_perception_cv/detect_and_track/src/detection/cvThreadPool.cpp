#include "defines.hpp"
#include "ConcurrentQueue.hpp"
#include "Settings.hpp"
#include "Camera.hpp"
#include "ArmorDetection.hpp"
#include "StopWatch.hpp"
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "cvThreadPool.hpp"
#include "main.hpp"

using namespace std;
using namespace cv;

ThreadPool::ThreadPool()
	: run(false), armorStoragesQ(5){};

ThreadPool::~ThreadPool()
{
	stopThreads();
	for (int i = 0; i < cams.size(); i++)
	{
		delete cams[i];
	}
};

/**
 * @brief startup boot sequence
 * this function tries to create objects for accessing and processing cameras and their inputs
 * specified by the global Setting object
 * return true if at least one camera can be initiallized
 *
 */
bool ThreadPool::initialize()
{
	startCams(detectionNodeShared::settings, cams);

	if (cams.size() < 1)
	{
		ROS_INFO("No proper camera configured found");
		return false;
	}
	return true;
};

void ThreadPool::doWork()
{
	if (detectionNodeShared::rosIntertface->tryProcess(armorStoragesQ, displayQ))
		return;


	if (ArmorProcessor::tryProcess(lightStoragesQ, armorStoragesQ))
		return;

	for (int i = 0; i < cams.size(); i++)
	{
		if (LightFinder::tryProcess(cams[i]->outQ, lightStoragesQ))
			return;
	}

	tryReadCam(detectionNodeShared::settings, cams);
};

void ThreadPool::worker()
{
	while (run)
	{
		doWork();
	}
};

/**
 * @brief work that the main thread should do, mainly debug
 */
void ThreadPool::doBossWork()
{
	static ArmorStorage *tempout = NULL;
	if (displayQ.dequeue(tempout))
	{
		if (detectionNodeShared::settings.Debug)
		{
			imshow(tempout->sourceCamPtr->getName(), tempout->img);
			imshow(tempout->sourceCamPtr->getName() + "preprocessedImg Blue",
				   tempout->preprocessedImgB);
			imshow(tempout->sourceCamPtr->getName() + "preprocessedImg Red",
				   tempout->preprocessedImgR);
			imshow(tempout->sourceCamPtr->getName() + "preprocessedImg OR",
				   tempout->preprocessedImgOR);
		}

		delete tempout;
	}

	int keyin = waitKey(1);

	switch (keyin)
	{
	case 32:
		if (this->run)
			this->stopThreads();
		else
			this->startThreads(detectionNodeShared::settings.threadCount);
		break;
	case 27:
		this->stopThreads();
		exit(0);
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
			workers.push_back(new thread(&ThreadPool::worker, this));

			cout << "created thread " << i << endl;
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
