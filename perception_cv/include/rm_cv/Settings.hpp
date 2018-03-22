#pragma once
#include "opencv2/core/core.hpp"
#include <string>

using namespace std;
using namespace cv;

#define MAX_CAM_COUNT 5
#define V4LDriver 0
#define ArmorColor_RED 0
#define ArmorColor_BLUE 1

bool fileExist(const std::string &filename);

namespace fsHelper
{
template <class T>
void readOrDefault(const FileNode &node, T &x, const T &default_value = T())
{
  if (node.empty())
    x = default_value;
  else
    node >> x;
}

void setDoubleToInt(int pos, void *doublePtr);
}

class CameraDeployConfig
{
public:
  CameraDeployConfig(){};
  int camDriver = 0;
  string camFileName = "";
  //cameras have same id means they are used together as a setup of stereo vision
  int stereoGp = -1;
  //the motion group where the camera is located, 0 for fixed in the chassis, 1 for on the gimbal, others to be defined
  int motionGroup = 0;

  void write(FileStorage &fs) const;
  void read(const FileNode &node);
};
void write(FileStorage &fs, const std::string &, const CameraDeployConfig &x);
void read(const FileNode &node, CameraDeployConfig &x, const CameraDeployConfig &default_value = CameraDeployConfig());

class ADSetting
{
public:
  int enemyColor = ArmorColor_RED;
  // cv::Scalar HSVMinBlue = Scalar(10, 0, 130);
  // cv::Scalar HSVMaxBlue = Scalar(60, 255, 255);
  // cv::Scalar HSVMinRed = Scalar(80, 0, 130);
  // cv::Scalar HSVMaxRed = Scalar(60, 255, 255);
  cv::Scalar BGRMinBlue = Scalar(180, 0, 0);
  cv::Scalar BGRMaxBlue = Scalar(255, 255, 255);
  cv::Scalar BGRMinRed = Scalar(0, 0, 180);
  cv::Scalar BGRMaxRed = Scalar(255, 255, 255);
  //Filter lights
  float light_max_aspect_ratio_ = 5;
  float light_min_area_ = 5;
  float light_max_angle_ = 30;

  //Filter ArmorLightGp
  float armor_max_angle_diff_ = 10;
  float armor_max_anglePos_diff_ = 10;
  float armor_min_area_ = 50;
  float armor_max_aspect_ratio_ = 2.5;
  float armor_max_stddev_ = 40;

  //geometry of the armor, upper-left, lower-left, upper-right, lower-right
  vector<Point3f> realArmorPoints = {Point3f(-65, -28, 0),
                                     Point3f(-65, 28, 0),
                                     Point3f(65, -28, 0),
                                     Point3f(65, 28, 0)};

  void write(FileStorage &fs) const;
  void read(const FileNode &node);
};
void write(FileStorage &fs, const std::string &, const ADSetting &x);
void read(const FileNode &node, ADSetting &x, const ADSetting &default_value = ADSetting());

class Settings
{
public:
  Settings(const std::string &filename);
  ~Settings();
  void save();
  void load();
  void openTuner(); //only use when this object is not being used

  CameraDeployConfig *cameraConfigs;
  ADSetting adSetting;

private:
  std::string filename;
};