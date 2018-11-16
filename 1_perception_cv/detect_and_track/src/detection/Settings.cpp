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
        fs["Debug"] >> Debug;
        fsHelper::readOrDefault(fs["threadCount"], threadCount, 2);
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
    fout << "threadCount" << threadCount;

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
    fsHelper::readOrDefault(node["stereoGp"], stereoGp, stereoGp);
    fsHelper::readOrDefault(node["motionGroup"], motionGroup, motionGroup);
}

void ADSetting::write(FileStorage &fs) const //Write serialization for this class
{
    fs << "{"
       << "EnemyColor" << enemyColor
       << "light_min_aspect_ratio_" << light_min_aspect_ratio_
       << "light_min_area_" << light_min_area_
       << "light_max_tilt_" << light_max_tilt_

       << "armor_max_tilt_diff_" << armor_max_tilt_diff_
       << "armor_max_anglePos_diff_" << armor_max_anglePos_diff_
       << "armor_min_area_" << armor_min_area_
       << "armor_max_aspect_ratio_" << armor_max_aspect_ratio_
       << "armor_min_aspect_ratio_" << armor_min_aspect_ratio_
       << "armor_max_light_length_diff_proportion_" << armor_max_light_length_diff_proportion_

       << "red_blue_classificatino_ratio" << red_blue_classificatino_ratio

       << "realArmorPoints" << realArmorPoints
       << "realArmorPoints_Big" << realArmorPoints_Big
       << "}";
};

void ADSetting::read(const FileNode &node) //Read serialization for this class
{
    fsHelper::readOrDefault(node["EnemyColor"], enemyColor, enemyColor);
    fsHelper::readOrDefault(node["light_min_aspect_ratio_"], light_min_aspect_ratio_, light_min_aspect_ratio_);
    fsHelper::readOrDefault(node["light_min_area_"], light_min_area_, light_min_area_);
    fsHelper::readOrDefault(node["light_max_tilt_"], light_max_tilt_, light_max_tilt_);
    fsHelper::readOrDefault(node["armor_max_tilt_diff_"], armor_max_tilt_diff_, armor_max_tilt_diff_);
    fsHelper::readOrDefault(node["armor_max_anglePos_diff_"], armor_max_anglePos_diff_, armor_max_anglePos_diff_);
    fsHelper::readOrDefault(node["armor_min_area_"], armor_min_area_, armor_min_area_);
    fsHelper::readOrDefault(node["armor_max_aspect_ratio_"], armor_max_aspect_ratio_, armor_max_aspect_ratio_);
    fsHelper::readOrDefault(node["armor_min_aspect_ratio_"], armor_min_aspect_ratio_, armor_min_aspect_ratio_);
    fsHelper::readOrDefault(node["armor_max_light_length_diff_proportion_"], armor_max_light_length_diff_proportion_, armor_max_light_length_diff_proportion_);
    fsHelper::readOrDefault(node["realArmorPoints"], realArmorPoints, realArmorPoints);
    fsHelper::readOrDefault(node["realArmorPoints_Big"], realArmorPoints_Big, realArmorPoints_Big);
    fsHelper::readOrDefault(node["red_blue_classificatino_ratio"], red_blue_classificatino_ratio, red_blue_classificatino_ratio);
};

void LightFilterSetting::write(FileStorage &fs) const
{
    fs << "{"
       << "UseHSV" << UseHSV
       << "morphoRadius" << morphoRadius
       << "BGRMinBlue" << BGRMinBlue
       << "BGRMaxBlue" << BGRMaxBlue
       << "BGRMinRed" << BGRMinRed
       << "BGRMaxRed" << BGRMaxRed

       << "HSVMinBlue" << HSVMinBlue
       << "HSVMaxBlue" << HSVMaxBlue
       << "HSVMinRed" << HSVMinRed
       << "HSVMaxRed" << HSVMaxRed
       << "}";
};

void LightFilterSetting::read(const FileNode &node) //Read serialization for this class
{
    fsHelper::readOrDefault(node["UseHSV"], UseHSV, UseHSV);
    fsHelper::readOrDefault(node["morphoRadius"], morphoRadius, morphoRadius);

    fsHelper::readOrDefault(node["BGRMinBlue"], BGRMinBlue, BGRMinBlue);
    fsHelper::readOrDefault(node["BGRMaxBlue"], BGRMaxBlue, BGRMaxBlue);
    fsHelper::readOrDefault(node["BGRMinRed"], BGRMinRed, BGRMinRed);
    fsHelper::readOrDefault(node["BGRMaxRed"], BGRMaxRed, BGRMaxRed);
    fsHelper::readOrDefault(node["HSVMinBlue"], HSVMinBlue, HSVMinBlue);
    fsHelper::readOrDefault(node["HSVMaxBlue"], HSVMaxBlue, HSVMaxBlue);
    fsHelper::readOrDefault(node["HSVMinRed"], HSVMinRed, HSVMinRed);
    fsHelper::readOrDefault(node["HSVMaxRed"], HSVMaxRed, HSVMaxRed);
};

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

void write(FileStorage &fs, const std::string &, const LightFilterSetting &x)
{
    x.write(fs);
}

void read(const FileNode &node, LightFilterSetting &x, const LightFilterSetting &default_value)
{
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}
