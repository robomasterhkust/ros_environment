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
#include "ArmorDetection.hpp"
#include "StopWatch.hpp"
#include "ROSInterface.hpp"
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
  ThreadPool();
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
  vector<Camera *> cams;

  ConcurrentQueue<LightStorage> lightStoragesQ;
  ConcurrentQueue<ArmorStorage> armorStoragesQ;
  ConcurrentQueue<ArmorStorage> displayQ;

  vector<thread *> workers;
};