/**
 * @brief This thing basically handle non-ui workflow
 * 
 * @file ThreadPool.hpp
 * @author Alex Au
 * @date 2018-04-20
 */

#include "ConcurrentQueue.hpp"
#include "Settings.hpp"
#include "Camera.hpp"
#include "V4LCamDriver.hpp"
#include "ArmorDetection.hpp"
#include "ArmorTracker.hpp"
#include "StopWatch.hpp"
#include "GimbalController.hpp"
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>

using namespace std;
using namespace cv;

class ThreadPool
{
public:
  ThreadPool(int argc, char **argv);
  ~ThreadPool();

  bool initialize();

  void doWork();

  void worker();

  //this should only be ran in the main thread
  void doBossWork();

  bool startThreads(const int &count);

  void stopThreads();

  volatile bool run;

private:
  void termP(const std_msgs::Empty &in);
  vector<Camera *> cams;

  //TODO: store storages in vector instead? try to be static lol
  vector<ConcurrentQueue<LightStorage> *> LightStorageQueues;
  vector<ConcurrentQueue<ArmorStorage> *> ArmorStorageQueues;

  vector<ConcurrentQueue<LightStorage> *> LightStorageQueues_debug;
  vector<ConcurrentQueue<ArmorStorage> *> ArmorStorageQueues_debug;

  vector<LightFinder *> lightfinders;
  vector<ArmorProcessor *> armorProcessors;
  ArmorTracker tracker;

  vector<thread *> workers;

  ros::NodeHandle *rosNodeHandle = NULL;
  ros::Subscriber *termsub;
  GimbalController *gctrl = NULL;
  //ROSHandle rosHandle;
};