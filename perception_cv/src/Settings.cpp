#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include "Settings.hpp"

using namespace std;
using namespace cv;

void fsHelper::setDoubleToInt(int pos, void *doublePtr)
{
    *((double *)doublePtr) = pos;
    cout << "setDoubleToInt\n";
}

Settings::Settings(const string &fn) : filename(fn), cameraConfigs(NULL)
{
}
Settings::~Settings()
{
    delete[] cameraConfigs;
}

void Settings::load()
{
    delete[] cameraConfigs;
    cameraConfigs = new CameraDeployConfig[MAX_CAM_COUNT];
    if (fileExist(filename))
    {
        cv::FileStorage fs(Settings::filename, cv::FileStorage::READ);
        //load AD setting
        read(fs["ADSetting"], adSetting);
        //load camera startup infos
        for (int i = 0; i < MAX_CAM_COUNT; i++)
        {
            if (!fs["Cam" + to_string(i)].empty())
            {
                read(fs["Cam" + to_string(i)], cameraConfigs[i]);
            }
        }

        fs.release();
    }
    else
    {
        //using default settings when no previous setting.xml exist
        adSetting = ADSetting();

        for (int i = 0; i < MAX_CAM_COUNT; i++)
        {
            cameraConfigs[i] = CameraDeployConfig();
        }
    }

    save();
}

void Settings::save()
{
    cv::FileStorage fout(Settings::filename, cv::FileStorage::WRITE);

    fout << "ADSetting" << adSetting;

    for (int i = 0; i < MAX_CAM_COUNT; i++)
    {
        fout << ("Cam" + to_string(i)) << cameraConfigs[i];
    }

    fout.release();
}

void Settings::openTuner()
{
    int tempBGRMinBlueH = adSetting.BGRMinBlue[0];
    int tempBGRMinBlueS = adSetting.BGRMinBlue[1];
    int tempBGRMinBlueV = adSetting.BGRMinBlue[2];
    int tempBGRMaxBlueH = adSetting.BGRMaxBlue[0];
    int tempBGRMaxBlueS = adSetting.BGRMaxBlue[1];
    int tempBGRMaxBlueV = adSetting.BGRMaxBlue[2];
    int tempBGRMinRedH = adSetting.BGRMinRed[0];
    int tempBGRMinRedS = adSetting.BGRMinRed[1];
    int tempBGRMinRedV = adSetting.BGRMinRed[2];
    int tempBGRMaxRedH = adSetting.BGRMaxRed[0];
    int tempBGRMaxRedS = adSetting.BGRMaxRed[1];
    int tempBGRMaxRedV = adSetting.BGRMaxRed[2];
    namedWindow("Setting Tuner");
    createTrackbar("Color: Blue - 1; Red - 0", "Setting Tuner", &this->adSetting.enemyColor, 1);
    createTrackbar("BGRMinBlueH", "Setting Tuner", &tempBGRMinBlueH, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMinBlue[0]);
    createTrackbar("BGRMaxBlueH", "Setting Tuner", &tempBGRMaxBlueH, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMaxBlue[0]);
    createTrackbar("BGRMinBlueS", "Setting Tuner", &tempBGRMinBlueS, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMinBlue[1]);
    createTrackbar("BGRMaxBlueS", "Setting Tuner", &tempBGRMaxBlueS, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMaxBlue[1]);
    createTrackbar("BGRMinBlueV", "Setting Tuner", &tempBGRMinBlueV, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMinBlue[2]);
    createTrackbar("BGRMaxBlueV", "Setting Tuner", &tempBGRMaxBlueV, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMaxBlue[2]);
    createTrackbar("BGRMinRedH", "Setting Tuner", &tempBGRMinRedH, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMinRed[0]);
    createTrackbar("BGRMaxRedH", "Setting Tuner", &tempBGRMaxRedH, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMaxRed[0]);
    createTrackbar("BGRMinRedS", "Setting Tuner", &tempBGRMinRedS, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMinRed[1]);
    createTrackbar("BGRMaxRedS", "Setting Tuner", &tempBGRMaxRedS, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMaxRed[1]);
    createTrackbar("BGRMinRedV", "Setting Tuner", &tempBGRMinRedV, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMinRed[2]);
    createTrackbar("BGRMaxRedV", "Setting Tuner", &tempBGRMaxRedV, 255, fsHelper::setDoubleToInt, &this->adSetting.BGRMaxRed[2]);
}

bool fileExist(const string &filename)
{
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

void CameraDeployConfig::write(FileStorage &fs) const
{
    fs << "{"
       << "camDriver" << camDriver
       << "camFileName" << camFileName
       << "stereoGp" << stereoGp
       << "motionGroup" << motionGroup
       << "}";
}

void CameraDeployConfig::read(const FileNode &node)
{
    node["camDriver"] >> camDriver;
    node["camFileName"] >> camFileName;
    node["stereoGp"] >> stereoGp;
    node["motionGroup"] >> motionGroup;
}

void ADSetting::write(FileStorage &fs) const //Write serialization for this class
{
    fs << "{"
       << "EnemyColor" << enemyColor
       << "BGRMinBlue" << BGRMinBlue
       << "BGRMaxBlue" << BGRMaxBlue
       << "BGRMinRed" << BGRMinRed
       << "BGRMaxRed" << BGRMaxRed

       << "light_max_aspect_ratio_" << light_max_aspect_ratio_
       << "light_min_area_" << light_min_area_
       << "light_max_angle_" << light_max_angle_

       << "armor_max_angle_diff_" << armor_max_angle_diff_
       << "armor_max_anglePos_diff_" << armor_max_anglePos_diff_
       << "armor_min_area_" << armor_min_area_
       << "armor_max_aspect_ratio_" << armor_max_aspect_ratio_
       << "armor_max_stddev_" << armor_max_stddev_

       << "realArmorPoints" << realArmorPoints
       << "}";
}
void ADSetting::read(const FileNode &node) //Read serialization for this class
{
    node["EnemyColor"] >> enemyColor;
    node["BGRMinBlue"] >> BGRMinBlue;
    node["BGRMaxBlue"] >> BGRMaxBlue;
    node["BGRMinRed"] >> BGRMinRed;
    node["BGRMaxRed"] >> BGRMaxRed;

    node["light_max_aspect_ratio_"] >> light_max_aspect_ratio_;
    node["light_min_area_"] >> light_min_area_;
    node["light_max_angle_"] >> light_max_angle_;

    node["armor_max_angle_diff_"] >> armor_max_angle_diff_;
    node["armor_max_anglePos_diff_"] >> armor_max_anglePos_diff_;
    node["armor_min_area_"] >> armor_min_area_;
    node["armor_max_aspect_ratio_"] >> armor_max_aspect_ratio_;
    node["armor_max_stddev_"] >> armor_max_stddev_;

    node["realArmorPoints"] >> realArmorPoints;
}

void write(FileStorage &fs, const std::string &, const CameraDeployConfig &x)
{
    x.write(fs);
}

void read(const FileNode &node, CameraDeployConfig &x, const CameraDeployConfig &default_value)
{
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}

void write(FileStorage &fs, const std::string &, const ADSetting &x)
{
    x.write(fs);
}

void read(const FileNode &node, ADSetting &x, const ADSetting &default_value)
{
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}