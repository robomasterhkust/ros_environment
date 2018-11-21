#pragma once
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "Settings.hpp"
#include "ConcurrentQueue.hpp"
#include "Camera.hpp"
#include <time.h>
#include <string>

//Handle LightGp finding, tracking, analysising and prediction
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
  /**
 * @brief Construct a new Light object
 * the rect will be smaller in dimension by 1 pixel
 * @param _rect
 */
  Light(RotatedRect &_rect, vector<cv::Point> &contour);
  RotatedRect rect;

  //TODO: use contour to find vertex instead
  Point2f vertex[2]; //0 is upper, 1 is lower

  /**
   * @brief test if the tool body belongs to this light, and update this light with tool,
   * tool should be deleted after the merging
   * the result of A.merge(B) should be same with B.merge(A)
   */
  bool merge(const Light &toolBody);

  float length() const;
};

class Armor //Armor detected from a camera, store real coordinates
{
public:
  Armor(const Vec3f &rotation,
        const Vec3f &translation,
        const cv::Matx33d &translationCov,
        vector<Point2f> vertices,
        bool isBig,
        bool isBlue);

  //attitude of armor, relative to camera's reference frame
  Vec3f translation;
  Vec3f rotation;

  cv::Matx33d translationCov;

  //axis angle direction of armor from camera
  Vec3f direction;
  vector<Point2f> vertices; //for debug purpose only
  bool isBig;
  bool isBlue;
};

class LightStorage : public FrameInfo
{
public:
  LightStorage(const Mat &_preprocessedImgR,
               const Mat &_preprocessedImgB,
               FrameInfo *const sourceFrameObj)
      : preprocessedImgR(_preprocessedImgR),
        preprocessedImgB(_preprocessedImgB),
        FrameInfo(*sourceFrameObj){};

  ~LightStorage();

  /**
 * @brief join collinear and adj. lights
 * not a fast operation, max O(n!)
 */
  void joinBrokenLights();
  //draw detected lights into the source frame image
  void drawLights();

  Mat preprocessedImgR;
  Mat preprocessedImgB;
  Mat preprocessedImgOR;
  vector<Light> lightsB;
  vector<Light> lightsR;
};

class ArmorStorage : public LightStorage
{
public:
  ArmorStorage(LightStorage *const sourceLights)
      : LightStorage(*sourceLights){};

  ~ArmorStorage();

  //print information of detected lights to the console
  void printArmors();
  //draw detected armors into the source frame image
  void drawArmors();

  vector<Armor> armors;
};

namespace LightFinder
{
bool tryProcess(ConcurrentQueue<FrameInfo> &inputQ, ConcurrentQueue<LightStorage> &outputQueue);
//private:
LightStorage *findLight(FrameInfo *const frame);

// LightStorage *findLightwithSetting(Mat image, LightFilterSetting *setting);

static void ConvertRRectAngle2Normal(RotatedRect &rRect);

static bool testAspectRatio(const RotatedRect &light);
static bool testArea(const RotatedRect &light);
static bool testTilt(const RotatedRect &light);
}; // namespace LightFinder

//in: lights found from different camera groups
//out: armor coordinates

namespace ArmorProcessor
{
bool tryProcess(ConcurrentQueue<LightStorage> &inputQ, ConcurrentQueue<ArmorStorage> &outputQueue);

//private:
ArmorStorage *generateArmors(LightStorage *const lights);

class LightGp
{
public:
  LightGp(const Light *firstlight);
  ~LightGp();
  void coutInfo() const;
  void paintOnMat(Mat &img, int num = -1) const;

  //store pointts of ligts, upper -> lower -> upper -> ... and from one side to another
  void getVertices(vector<Point2f> &points) const;

  //get the maximum light separation / mean light length ratio between lights in the group
  float getMaxRatio() const;

  void addLight(const Light *const newlight);
  bool match(const Light &light);
  void merge(LightGp &toolArmorLightGp);

  vector<const Light *> lights;
  vector<float> relativePositions;

private:
  //heading: a vector in the direction of lights
  Vec2f heading;
  //perpendicular to heading
  Vec2f normal;
};

/**
 * @brief given the reference to a lightStorage, group the lights together by possible formulation of armor plates
 */
void armorGrouper(const vector<Light> &srcLights, vector<LightGp> &dstLightGps);

/**
 * @brief break light groups if something of different color is in between
 */
void armorBreaker(vector<LightGp> &BLightGps, vector<LightGp> &RLightGps);
//finding coordinates
void armorLocator(const vector<LightGp> &ArmorLightGps,
                  bool isBlue,
                  const Camera *const &sourceCamPtr,
                  ArmorStorage &result);

//testSeparation: test by maximum separation
bool testSeparation(const Light *a, const Light *b);

//check both the orientation of lights to see if they are in same direction
bool testHeading(const Light *a, const Light *b);

//test the angle formed between lights and the connecting line
bool testTilt(const Light *a, const Light *b);

//deplete?
bool testSize(const Light *a, const Light *b);
bool testLength(const Light *a, const Light *b);
}; // namespace ArmorProcessor
