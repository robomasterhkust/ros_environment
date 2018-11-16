#pragma once
#include "opencv2/core/core.hpp"
#include <string>
#include "defines.hpp"

using namespace std;
using namespace cv;

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
} // namespace fsHelper

class CameraDeployConfig
{
public:
  CameraDeployConfig(){};
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
  bool showPreview = true;
  int enemyColor = ArmorColor_RED;

  //Filter lights
  float light_min_aspect_ratio_ = ARMOR_LIGHT_MIN_RATIO;
  float light_min_area_ = ARMOR_LIGHT_MIN_AREA;
  float light_max_tilt_ = ARMOR_LIGHT_MAX_TILT;

  //Filter ArmorLightGp
  float armor_max_tilt_diff_ = ARMOR_GROUPING_MAX_TILT_DIFF;
  float armor_max_anglePos_diff_ = ARMOR_GROUPING_MAX_ANGULAR_POS_DIFF;
  float armor_max_light_length_diff_proportion_ = ARMOR_GROUPING_MAX_LIGHT_LENGTH_DIFF;
  float armor_min_area_ = ARMOR_GROUPING_MIN_AREA;
  float armor_max_aspect_ratio_ = ARMOR_GROUPING_MAX_ASPECT_RATIO;
  float armor_min_aspect_ratio_ = ARMOR_GROUPING_MIN_ASPECT_RATIO;
  float red_blue_classificatino_ratio = 1.5;

  //geometry of the armor, upper-left, lower-left, upper-right, lower-right
  vector<Point3f> realArmorPoints = {Point3f(-SMALL_ARMOR_WIDTH / 2, -ALL_ARMOR_HEIGHT / 2, 0),
                                     Point3f(-SMALL_ARMOR_WIDTH / 2, ALL_ARMOR_HEIGHT / 2, 0),
                                     Point3f(SMALL_ARMOR_WIDTH / 2, -ALL_ARMOR_HEIGHT / 2, 0),
                                     Point3f(SMALL_ARMOR_WIDTH / 2, ALL_ARMOR_HEIGHT / 2, 0)};

  vector<Point3f> realArmorPoints_Big = {Point3f(-BIG_ARMOR_WIDTH / 2, -ALL_ARMOR_HEIGHT / 2 / 2, 0),
                                         Point3f(-BIG_ARMOR_WIDTH / 2, ALL_ARMOR_HEIGHT / 2 / 2, 0),
                                         Point3f(BIG_ARMOR_WIDTH / 2, -ALL_ARMOR_HEIGHT / 2 / 2, 0),
                                         Point3f(BIG_ARMOR_WIDTH / 2, ALL_ARMOR_HEIGHT / 2 / 2, 0)};

  void write(FileStorage &fs) const;
  void read(const FileNode &node);
};
void write(FileStorage &fs, const std::string &, const ADSetting &x);
void read(const FileNode &node, ADSetting &x, const ADSetting &default_value = ADSetting());

//per camera setting for light finder to filter out correct colors
class LightFilterSetting
{
public:
  cv::Vec3i BGRMinBlue = ARMOR_LIGHT_BGRMinBlue;
  cv::Vec3i BGRMaxBlue = ARMOR_LIGHT_BGRMaxBlue;
  cv::Vec3i BGRMinRed = ARMOR_LIGHT_BGRMinRed;
  cv::Vec3i BGRMaxRed = ARMOR_LIGHT_BGRMaxRed;

  cv::Vec3i HSVMinBlue = ARMOR_LIGHT_HSVMinBlue;
  cv::Vec3i HSVMaxBlue = ARMOR_LIGHT_HSVMaxBlue;
  cv::Vec3i HSVMinRed = ARMOR_LIGHT_HSVMinRed;
  cv::Vec3i HSVMaxRed = ARMOR_LIGHT_HSVMaxRed;

  cv::Vec3i BGRMinWhite;
  cv::Vec3i BGRMaxWhite;

  int UseHSV = 0;
  int morphoRadius = 2;

  void write(FileStorage &fs) const;
  void read(const FileNode &node);
};
void write(FileStorage &fs, const std::string &, const LightFilterSetting &x);
void read(const FileNode &node, LightFilterSetting &x, const LightFilterSetting &default_value = LightFilterSetting());

class Settings
{
public:
  Settings(const std::string &filename);
  ~Settings();
  void save();
  void load();

  CameraDeployConfig *cameraConfigs;
  ADSetting adSetting;
  bool Debug = true;
  int threadCount = 2;

private:
  std::string filename;
};