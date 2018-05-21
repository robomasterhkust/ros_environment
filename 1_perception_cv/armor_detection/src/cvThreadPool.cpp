#include "defines.hpp"
#include "ConcurrentQueue.hpp"
#include "Settings.hpp"
#include "Camera.hpp"
#include "ArmorDetection.hpp"
#include "ArmorTracker.hpp"
#include "StopWatch.hpp"
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "cvThreadPool.hpp"

using namespace std;
using namespace cv;

extern Settings settings;

ThreadPool::ThreadPool(int argc, char **argv)
	: run(false)
{
	ros::init(argc, argv, "CV_node");
	rosNodeHandle = new ros::NodeHandle;
};

ThreadPool::~ThreadPool()
{
	stopThreads();
	for (int i = 0; i < cams.size(); i++)
	{
		delete cams[i];
	}
	for (auto i : lightfinders)
		delete i;
	for (auto i : armorProcessors)
		delete i;
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
	//Subscriber for termination call
	termsub = new ros::Subscriber(rosNodeHandle->subscribe(TOPIC_NAME_TERMINATE, 5, &ThreadPool::termP, this));

	gctrl = new GimbalController(rosNodeHandle, settings.enable_Serial);
	//TODO: automate initiallize squence
#ifdef DEBUG
	startCams(settings, cams, 2);
#else
	startCams(settings, cams, 1);
#endif
	if (cams.size() < 1)
	{
		ROS_INFO("No proper camera configured found");
		return false;
	}
	int successCamCount = 0;
	for (int i = 0; i < cams.size(); i++)
	{
		//each light finder and armor processor should only process from one image source for now
		lightfinders.push_back(new LightFinder(&cams[i]->outQ[0]));

		LightStorageQueues.push_back(new ConcurrentQueue<LightStorage>());
		LightStorageQueues_debug.push_back(new ConcurrentQueue<LightStorage>());
		lightfinders.back()->addOutputQueue(LightStorageQueues.back());
		lightfinders.back()->addOutputQueue(LightStorageQueues_debug.back());

		armorProcessors.push_back(new ArmorProcessor(LightStorageQueues.back()));

		ArmorStorageQueues.push_back(new ConcurrentQueue<ArmorStorage>());
		ArmorStorageQueues_debug.push_back(new ConcurrentQueue<ArmorStorage>());
		armorProcessors.back()->addOutputQueue(ArmorStorageQueues.back());
		armorProcessors.back()->addOutputQueue(ArmorStorageQueues_debug.back());

		tracker.addSource(ArmorStorageQueues.back());
	}

	return true;
};

void ThreadPool::termP(const std_msgs::Empty &in)
{
	throw;
}

void ThreadPool::doWork()
{
	//try to do the most important things first
	if (tracker.tryWork())
		return;
	for (auto i : armorProcessors)
	{
		if (i->tryWork())
			return;
	}
	for (auto j : lightfinders)
	{
		if (j->tryWork())
			return;
	}
	tryReadCam(settings, cams);
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
#ifdef DEBUG
	FrameInfo *tempOut = NULL;
	LightStorage *tempLS = NULL;
	ArmorStorage *tempAS = NULL;

	for (auto i : cams)
	{
		if (i->outQ[1].dequeue(tempOut))
		{
			imshow("frame", tempOut->img);
			delete tempOut;
		}
	}

	for (int i = 0; i < cams.size(); i++)
	{
		if (LightStorageQueues_debug[i]->dequeue(tempLS))
		{
			tempLS->draw();
			delete tempLS;
		}
	}

	for (int i = 0; i < cams.size(); i++)
	{
		if (ArmorStorageQueues_debug[i]->dequeue(tempAS))
		{
			tempAS->draw();
			delete tempAS;
		}
	}
#endif

	timespec uptime;
	timeval tnow;
	clock_gettime(CLOCK_MONOTONIC, &uptime);
	tnow.tv_sec = uptime.tv_sec;
	tnow.tv_usec = uptime.tv_nsec / 1000;

	cv::Vec3d targetCoor = {0, 0, 0};

	vector<TrackedArmor> tempTrackedArmor;

	//set targetCoor to the closest armor, more intelligence to be introduced
	tracker.getValidatedTargets(tempTrackedArmor);
	if (tempTrackedArmor.size() > 0)
		tempTrackedArmor.front().predict(
			tempTrackedArmor.front().getLastSeen(),
			targetCoor);
	//TODO: intelligence
	//rosHandle.publishCoor(targetCoor);
	gctrl->updateTarget(targetCoor[0], targetCoor[1], targetCoor[2]);
	gctrl->publishToCan();
	ros::spinOnce();
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