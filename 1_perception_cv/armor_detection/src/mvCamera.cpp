#include "mvCamera.hpp"
#include "CameraApi.h"
#include "CameraDefine.h"
#include "CameraStatus.h"
#include <iostream>
#include <chrono>
#include <ctime>

using namespace cv;

mvCamera::mvCamera(int outCount, const string &config_path)
	: Camera(outCount, config_path){};

mvCamera::~mvCamera()
{
	CameraUnInit(hCamera);
	//Caution! uninit before free
	//free(g_pRgbBuffer);
};

bool mvCamera::initialize()
{
	int error_no;
	if ((error_no = CameraSdkInit(1)) != CAMERA_STATUS_SUCCESS)
	{
		cout << "ERROR: CameraSdkInit returned " << error_no << endl;
		return false;
	}

	CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
	if (iCameraCounts == 0)
	{
		cout << "ERROR: No mv camera found" << endl;
		return false;
	}
	iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
	if (iStatus != CAMERA_STATUS_SUCCESS)
	{
		return false;
	}

	this->applySetting();

	//CameraLoadParameter(hCamera, PARAMETER_TEAM_A);
	//sync the time
	timespec uptime;
	if (CameraRstTimeStamp(hCamera) != CAMERA_STATUS_SUCCESS)
		std::cout << "MindVision cam RstTimeStamp Failed!!!!!!\n";

	clock_gettime(CLOCK_MONOTONIC, &uptime);
	camSetTime.tv_sec = uptime.tv_sec;
	camSetTime.tv_usec = uptime.tv_nsec / 1000;

	info();

	return true;
}

void mvCamera::discardFrame()
{
	FrameInfo *pFrame = new FrameInfo(this);
	CameraSdkStatus temp;
	if ((temp = CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000)) == CAMERA_STATUS_SUCCESS)
	{
		CameraReleaseImageBuffer(hCamera, pbyBuffer);
	}
	else
	{
		cout << "\n\nmvCamera::getFrame error!! no:" << temp << endl;
	}
};

FrameInfo *mvCamera::getFrame()
{
	FrameInfo *pFrame = new FrameInfo(this);
	CameraSdkStatus temp;
	if ((temp = CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000)) == CAMERA_STATUS_SUCCESS)
	{
		pFrame->img = Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3);
		//directly write to mat's data location so that the mat object will own the data and delete for me
		CameraImageProcess(hCamera, pbyBuffer, pFrame->img.ptr(), &sFrameInfo);
		cvtColor(pFrame->img, pFrame->img, COLOR_RGB2BGR);
		CameraReleaseImageBuffer(hCamera, pbyBuffer);

		pFrame->rotationVec = this->rotationVec;
		pFrame->translationVec = this->translationVec;
		//uiTimeStamp is in 0.1ms from reset
		pFrame->capTime.tv_usec = camSetTime.tv_usec + sFrameInfo.uiTimeStamp * 100;
		pFrame->capTime.tv_sec = camSetTime.tv_sec + pFrame->capTime.tv_usec / 1000000;
		pFrame->capTime.tv_usec %= 1000000;
		return pFrame;
	}
	else
	{
		cout << "\n\nmvCamera::getFrame error!! no:" << temp << endl;
		return NULL;
	}
}

bool mvCamera::loadDriverParameters(const FileStorage &fs)
{
	const FileNode &node = fs["mvCamera"];
	fsHelper::readOrDefault(node["auto_exp"], auto_exp, false);
	fsHelper::readOrDefault(node["exposure_time"], exposure_time, 70.0);
	fsHelper::readOrDefault(node["analogGain"], analogGain, 0);
	fsHelper::readOrDefault(node["iGamma"], iGamma, 100);
	fsHelper::readOrDefault(node["iContrast"], iContrast, 100);
	fsHelper::readOrDefault(node["iSaturation"], iSaturation, 100);
	fsHelper::readOrDefault(node["iFrameSpeed"], iFrameSpeed, 1);
	fsHelper::readOrDefault(node["frameWidth"], tImageResolution.iWidth, 640);
	fsHelper::readOrDefault(node["frameHeight"], tImageResolution.iHeight, 480);
	return true;
};

bool mvCamera::storeDriverParameters(FileStorage &fs)
{
	fs << "mvCamera"
	   << "{"
	   << "auto_exp" << auto_exp
	   << "exposure_time" << exposure_time
	   << "analogGain" << analogGain
	   << "iGamma" << iGamma
	   << "iContrast" << iContrast
	   << "iSaturation" << iSaturation
	   << "iFrameSpeed" << iFrameSpeed
	   << "frameWidth" << tImageResolution.iWidth
	   << "frameHeight" << tImageResolution.iHeight
	   << "}";
};

bool mvCamera::writeCamConfig()
{
	CameraGetCapability(hCamera, &tCapability);

	CameraLoadParameter(hCamera, PARAMETER_TEAM_A);

	if (tCapability.sIspCapacity.bMonoSensor)
	{
		channel = 1;
		CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
	}
	else
	{
		channel = 3;
		CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
	}

	if (CameraSetAeState(hCamera, auto_exp) != CAMERA_STATUS_SUCCESS)
		std::cout << "MindVision cam CameraSetAeState Failed!!!!!!\n";

	if (CameraSetExposureTime(hCamera, exposure_time) != CAMERA_STATUS_SUCCESS)
		std::cout << "MindVision cam CameraSetExposureTime Failed!!!!!!\n";

	if (CameraSetAnalogGain(hCamera, analogGain) != CAMERA_STATUS_SUCCESS)
		std::cout << "MindVision cam CameraSetAnalogGain Failed!!!!!!\n";

	if (CameraSetGamma(hCamera, iGamma) != CAMERA_STATUS_SUCCESS)
		std::cout << "MindVision cam CameraSetGamma Failed!!!!!!\n";

	if (CameraSetContrast(hCamera, iContrast) != CAMERA_STATUS_SUCCESS)
		std::cout << "MindVision cam CameraSetContrast Failed!!!!!!\n";

	if (CameraSetSaturation(hCamera, iSaturation) != CAMERA_STATUS_SUCCESS)
		std::cout << "MindVision cam CameraSetSaturation Failed!!!!!!\n";

	if (CameraGetImageResolution(hCamera, &tImageResolution) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get image resolutiion failed\n";
	if (CameraSetImageResolution(hCamera, &tImageResolution) != CAMERA_STATUS_SUCCESS)
		std::cout << "MindVision cam CameraSetImageResolution Failed!!!!!!\n";

	// if (CameraSetFrameSpeed(hCamera, iFrameSpeed) != CAMERA_STATUS_SUCCESS)
	// 	std::cout << "MindVision cam CameraSetFrameSpeed Failed!!!!!!\n";
	//CameraLoadParameter(hCamera, PARAMETER_TEAM_A);
	return true;
};

bool mvCamera::readCamConfig()
{
	if (CameraGetImageResolution(hCamera, &tImageResolution) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get image resolutiion failed\n";
	if (CameraGetFrameSpeed(hCamera, &tFrameSpeed.iIndex) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get tFrameSpeed failed\n";
	if (CameraGetExposureTime(hCamera, &exposure_time) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get exposure time failed\n";
	if (CameraGetAeState(hCamera, (int *)&auto_exp) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get auto exposure enable failed\n";
	if (CameraGetSaturation(hCamera, &iSaturation) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get saturation failed\n";
	if (CameraGetContrast(hCamera, &iContrast) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get contrast failed\n";
	if (CameraGetGamma(hCamera, &iGamma) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get gamma failed\n";
	if (CameraGetAnalogGain(hCamera, &analogGain) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get analogGain failed\n";
	return true;
};

bool mvCamera::startStream()
{
	cout << "mvCamera::startStream\n";
	if (CameraPlay(hCamera))
	{
		cout << "mvCamera::startStream CameraPlay failed\n";
		return false;
	}
	return true;
};

bool mvCamera::closeStream()
{
	if (CameraPause(hCamera))
	{
		cout << "mvCamera::closeStream CameraPause failed\n";
		return false;
	}
	return true;
};

void mvCamera::info()
{
	CameraGetCapability(hCamera, &tCapability);
	if (CameraGetImageResolution(hCamera, &tImageResolution) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get image resolutiion failed\n";
	cout << "Initiallized image resolution: " << tImageResolution.iWidth << "x" << tImageResolution.iHeight << endl;

	if (CameraGetFrameSpeed(hCamera, &tFrameSpeed.iIndex) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get image resolutiion failed\n";
	std::cout << "Initiallized frame speed: " << tFrameSpeed.iIndex << endl;

	if (CameraGetExposureTime(hCamera, &exposure_time) != CAMERA_STATUS_SUCCESS)
		std::cout << "ERROR: mv cam get exposure time failed\n";
	std::cout << "Initiallized exposure time: " << exposure_time << "us" << endl;

	cout << "\nAvailable resolutions:\n";
	for (int i = 0; i < tCapability.iImageSizeDesc; i++)
	{
		cout << tCapability.pImageSizeDesc[i].acDescription << endl
			 << "iHeight: " << tCapability.pImageSizeDesc[i].iHeight << endl
			 << "iWidth: " << tCapability.pImageSizeDesc[i].iWidth << endl
			 << endl;
	}
};

void mvCamera::showSettigsPage()
{
	char temp[] = "mvcamsetting";
	CameraCreateSettingPage(hCamera, NULL, temp, NULL, NULL, 0);
	CameraShowSettingPage(hCamera, true);
};

bool mvCamera::getVideoSize(int &width, int &height)
{
	tSdkImageResolution *IR;
	if (CameraGetImageResolution(hCamera, IR) == CAMERA_STATUS_SUCCESS)
	{
		width = IR->iWidth;
		height = IR->iHeight;
		return true;
	}
	else
		return false;
}
bool mvCamera::setExposureTime(bool auto_exp, int t)
{
	if (auto_exp)
	{
		if (CameraSetAeState(hCamera, true) != CAMERA_STATUS_SUCCESS)
		{
			return false;
		}
	}
	else
	{
		CameraSetAeState(hCamera, false);
		if (CameraSetExposureTime(hCamera, (double)t) != CAMERA_STATUS_SUCCESS)
		{
			return false;
		}
	}
	return true;
}

bool mvCamera::readPreset(const int &settingGroup)
{
	if (CameraLoadParameter(hCamera, settingGroup) != CAMERA_STATUS_SUCCESS)
	{
		return false;
	}
	return true;
}
bool mvCamera::savePreset(const int &settingGroup)
{
	if (CameraSaveParameter(hCamera, settingGroup) != CAMERA_STATUS_SUCCESS)
	{
		return false;
	}
	return true;
}