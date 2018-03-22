#pragma once
#include <opencv2/opencv.hpp>
#include "Settings.hpp"
#include "ConcurrentQueue.hpp"
#include "Camera.hpp"
#include <time.h>

//Handle ArmorLightGp finding, tracking, analysising and prediction
//basically all the things apart from camera handling and ROS
//provide interface to the main cv software controller

//flow:
//camera source(s)                              -control fps, exposure etc?
//  | [image+time+cam_geometry]
//light finder (+ stereo distance finder)       -switch ROI and on/off
//  | [lights(rect)+Processedimage+time+cam_geometry (+ distance)]
//armor processor                               -info: camera's position and rotation in world's frame needed
//  | [armor info+ time]
//armor tracker  <-> [tracked armors]           -armor appear duration, probability, velocity etcF

using namespace std;
using namespace cv;

class Light
{
public:
  Light(RotatedRect &_rect);
  RotatedRect rect;
  Point2f vertex[2]; //0 is upper, 1 is lower
};

class Armor //Armor detected from a camera, store real coordinates
{
public:
  Armor(const Vec3f &rotation,
        const Vec3f &translation,
        vector<Point2f> vertices);
  Vec3f rotation;
  Vec3f translation;
  vector<Point2f> vertices; //for debug purpose only
};

class TrackedArmor
{
public:
  Vec3f rotation;
  Vec3f translation;
  Vec3f velocity;
  Vec3f angularVelocity;
  time_t duration;
  time_t lastSeen;
};

class LightStorage
{
public:
  LightStorage(const timeval &_capTime,
               const Vec3f &_rotationVec,
               const Vec3f &_translationVec,
               const Mat &_preprocessedImg,
               const Camera *sourceCamPtr)
      : capTime(_capTime),
        rotationVec(_rotationVec),
        translationVec(_translationVec),
        preprocessedImg(_preprocessedImg),
        sourceCamPtr(sourceCamPtr){};
  Mat preprocessedImg;
  vector<Light> lights;
  const timeval capTime;
  const Vec3f rotationVec;
  const Vec3f translationVec;

  const Camera *sourceCamPtr;

  void draw();
};

class ArmorStorage
{
public:
  ArmorStorage(const timeval &_capTime, const Size &imgSize)
      : capTime(_capTime),
        imgSize(imgSize){};

  void printArmors();
  void draw();

  vector<Armor> armors;
  const timeval capTime;
  const Size imgSize; //only used for debug
};

class TrackedArmorStorage
{
public:
  TrackedArmorStorage(){};
  void printArmors();
  vector<TrackedArmor> tArmors;
};

class LightFinder
{
public:
  LightFinder(ConcurrentQueue<FrameInfo> *_inputQ, int outQCount = 1, int outBufSize = 2);
  bool tryWork();
  ConcurrentQueue<LightStorage> **outputQArr;
  const int outQCount;

private:
  LightStorage *findLight(const FrameInfo &frame);

  static void ConvertRRectAngle2Normal(RotatedRect &rRect);

  static bool testAspectRatio(const RotatedRect &light);
  static bool testArea(const RotatedRect &light);
  static bool testAngle(const RotatedRect &light);

  ConcurrentQueue<FrameInfo> *inputQ;
  mutex lock;
};

//in: lights found from different camera groups
//out: armor coordinates

class ArmorProcessor
{
public:
  ArmorProcessor(ConcurrentQueue<LightStorage> *_inputQ, int outQCount = 1, int outBufSize = 2);
  bool tryWork();

  const int outQCount;
  ConcurrentQueue<ArmorStorage> **outputQArr;

private:
  ArmorStorage *generateArmors(const LightStorage &lights);

  class ArmorLightGp
  {
  public:
    ArmorLightGp(const Light *firstlight);
    ~ArmorLightGp();
    void coutInfo() const;
    void paintOnMat(Mat &img);

    //store pointts of ligts, upper -> lower -> upper -> ...
    void getVertices(vector<Point2f> &points) const;
    vector<const Light *> lights;
  };

  //grouping
  void armorGrouper(const LightStorage &lights, vector<ArmorLightGp> &ArmorLightGps);
  //finding coordinates
  void armorLocator(const vector<ArmorLightGp> &ArmorLightGps, const Mat cameraMatrix, const Mat distCoeffs, ArmorStorage &result);

  bool addLightToArmor(ArmorLightGp &gp, const Light *const newlight);
  void merge(ArmorLightGp &mainArmorLightGp, ArmorLightGp &toolArmorLightGp);
  bool matchLightWithArmor(const ArmorLightGp &gp, const Light &light) const;
  bool testSeparation(const Light *a, const Light *b) const;
  bool testTilt(const Light *a, const Light *b) const; //check both the orientation of lights, and the angle formed between them
  bool testSize(const Light *a, const Light *b) const;
  //TODO: test std deviation and darkness between lights?

  ConcurrentQueue<LightStorage> *inputQ;
  mutex lock;
};

class ArmorTracker //perform logging and tracking
{
public:
  ArmorTracker(TrackedArmorStorage *storage);
  void update(ArmorStorage &newArmors);
  void getMostProbableTarget(Armor &target);

private:
  TrackedArmorStorage *storage;
};
