#include "opencv2/core/core.hpp"
#include <string>
using namespace std;
using namespace cv;

class CameraSetting
{
public:
	CameraSetting(const string& fn) :filename1(fn) {}
	string device = "/dev/video0";
	int exposureTime = 30;
	int width = 1280,
		height = 720,
		fps = 30;
	cv::Mat cameraMatrix = (Mat1d(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	cv::Mat distortionMatrix = (Mat1d(1, 4) << 0, 0, 0, 0);
	string filename1;
	void read(const FileNode &node);
};

class SmallRuneSetting
{
	public:
		int width;
		int height;
		void read(const FileNode &node);
};

class ShooterSetting
{
	public:
		double speed;
		double Xoffset;
		double Yoffset;
		double Zoffset;
		void read(const FileNode &node);
};

class Settings
{
public:
	Settings(const string &filename1, const string& filename2) :filename1(filename1), filename2(filename2), cameraSetting(filename2) {}
	~Settings() {}
	bool load();

	CameraSetting cameraSetting;
	SmallRuneSetting smallRuneSetting;
	ShooterSetting shooterSetting;
	std::string filename1, filename2;     // filename1 is the file for setting, filename2 is the file for seeting cameraMatrix & distortionMatrix
	bool fileExist(const std::string &filename);
};


