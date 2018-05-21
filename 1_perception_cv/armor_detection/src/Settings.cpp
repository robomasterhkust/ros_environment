#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include "Settings.hpp"
#include "ros/ros.h"

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
    printf("Loading setting files from $ROS_HOME directory...\n");
    delete[] cameraConfigs;
    cameraConfigs = new CameraDeployConfig[MAX_CAM_COUNT];
    if (fileExist(filename))
    {
        cv::FileStorage fs(Settings::filename, cv::FileStorage::READ);

        fs["EnableSerialCom"] >> enable_Serial;
        fs["Debug"] >> Debug;
        //load AD setting
        read(fs["ADSetting"], adSetting);
        //load camera startup infos
        for (int i = 0; i < MAX_CAM_COUNT; i++)
        {
            if (!fs["Cam" + to_string(i)].empty())
            {
                read(fs["Cam" + to_string(i)], cameraConfigs[i]);
                cout << "read cam declaration " << i << " config file:" << cameraConfigs[i].camFileName << endl;
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
    fout << "Debug" << Debug;
    fout << "EnableSerialCom" << enable_Serial;

    fout << "ADSetting" << adSetting;

    for (int i = 0; i < MAX_CAM_COUNT; i++)
    {
        fout << ("Cam" + to_string(i)) << cameraConfigs[i];
    }

    fout.release();
}

bool fileExist(const string &filename)
{
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

void CameraDeployConfig::write(FileStorage &fs) const
{
    fs << "{"
       << "camFileName" << camFileName
       << "stereoGp" << stereoGp
       << "motionGroup" << motionGroup
       << "}";
}

void CameraDeployConfig::read(const FileNode &node)
{
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

       << "light_min_aspect_ratio_" << light_min_aspect_ratio_
       << "light_min_area_" << light_min_area_
       << "light_max_tilt_" << light_max_tilt_

       << "armor_max_tilt_diff_" << armor_max_tilt_diff_
       << "armor_max_anglePos_diff_" << armor_max_anglePos_diff_
       << "armor_min_area_" << armor_min_area_
       << "armor_max_aspect_ratio_" << armor_max_aspect_ratio_

       << "realArmorPoints" << realArmorPoints
       << "realArmorPoints_Big" << realArmorPoints_Big
       << "}";
}
void ADSetting::read(const FileNode &node) //Read serialization for this class
{
    node["EnemyColor"] >> enemyColor;
    node["BGRMinBlue"] >> BGRMinBlue;
    node["BGRMaxBlue"] >> BGRMaxBlue;
    node["BGRMinRed"] >> BGRMinRed;
    node["BGRMaxRed"] >> BGRMaxRed;

    node["light_min_aspect_ratio_"] >> light_min_aspect_ratio_;
    node["light_min_area_"] >> light_min_area_;
    node["light_max_tilt_"] >> light_max_tilt_;

    node["armor_max_tilt_diff_"] >> armor_max_tilt_diff_;
    node["armor_max_anglePos_diff_"] >> armor_max_anglePos_diff_;
    node["armor_min_area_"] >> armor_min_area_;
    node["armor_max_aspect_ratio_"] >> armor_max_aspect_ratio_;

    node["realArmorPoints"] >> realArmorPoints;
    node["realArmorPoints_Big"] >> realArmorPoints_Big;
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