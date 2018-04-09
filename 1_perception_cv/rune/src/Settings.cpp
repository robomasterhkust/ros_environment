#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <sys/stat.h>
#include <string>
#include <fstream>
#include "Settings.h"

using namespace std;
using namespace cv;

void CameraSetting::read(const FileNode &node) //Read serialization for this class
{
	node["exposureTime"] >> exposureTime;
	node["width"] >> width;
	node["height"] >> height;
	node["fps"] >> fps;

	FileStorage fs1;
	fs1.open(filename1, FileStorage::READ);

	fs1["camera_matrix"] >> cameraMatrix;
	fs1["distortion_coefficients"]>> distortionMatrix;

	fs1.release();
}
void SmallRuneSetting::read(const FileNode &node) //Read serialization for this class
{
	node["width"] >> width;
	node["height"] >> height;
}
void ShooterSetting::read(const FileNode &node)
{
	node["speed"]>>speed;
	node["Xoffset"]>>Xoffset;
	node["Yoffset"]>>Yoffset;
	node["Zoffset"]>>Zoffset;

}

bool Settings::load()
{
	if(!fileExist(filename1))
	{
		cout<<" NO " << filename1<<endl;
	}	
	if(!fileExist(filename2))
	{
		cout<<" NO " << filename2<<endl;
	}
	if (fileExist(filename1)&&fileExist(filename2))
	{
		cv::FileStorage fin(Settings::filename1, cv::FileStorage::READ);
		cameraSetting.read(fin["cameraSetting"]);
		smallRuneSetting.read(fin["smallRuneSetting"]);
		shooterSetting.read(fin["shooterSetting"]);
		cout<<"successfully load"<<endl;
		fin.release();
		return true;
	}
	else
	{
		return false;
	}
}

bool Settings::fileExist(const string &filename)
{
	struct stat buffer;
	return (stat(filename.c_str(), &buffer) == 0);
}
