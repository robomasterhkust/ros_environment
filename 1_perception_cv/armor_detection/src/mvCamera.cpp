#include "mvCamera.hpp"
#include "CameraApi.h"
#include "CameraDefine.h"
#include "CameraStatus.h"
#include <iostream>
#include <chrono>
#include <ctime>

using namespace cv;

mvCamera::mvCamera(int outCount) : Camera(outCount){};
mvCamera::mvCamera(int outCount, const FileStorage &fs) : Camera(outCount)
{
	loadBaseParameters(fs);
	loadDriverParameters(fs);
};

mvCamera::~mvCamera()
{
	CameraUnInit(hCamera);
	//Caution! uninit before free
	free(g_pRgbBuffer);
};

bool mvCamera::initialize()
{
	CameraSdkInit(1);
	CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
	if (iCameraCounts == 0)
	{
		return false;
	}
	iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
	if (iStatus != CAMERA_STATUS_SUCCESS)
	{
		return false;
	}
	CameraGetCapability(hCamera, &tCapability);
	g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);

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
		std::cout << "800RMB cam CameraSetAeState Failed!!!!!!\n";

	if (CameraSetExposureTime(hCamera, exposure_time) != CAMERA_STATUS_SUCCESS)
		std::cout << "800RMB cam CameraSetExposureTime Failed!!!!!!\n";

	if (CameraSetAnalogGain(hCamera, analogGain) != CAMERA_STATUS_SUCCESS)
		std::cout << "800RMB cam CameraSetAnalogGain Failed!!!!!!\n";

	if (CameraSetGamma(hCamera, iGamma) != CAMERA_STATUS_SUCCESS)
		std::cout << "800RMB cam CameraSetGamma Failed!!!!!!\n";

	if (CameraSetContrast(hCamera, iContrast) != CAMERA_STATUS_SUCCESS)
		std::cout << "800RMB cam CameraSetContrast Failed!!!!!!\n";

	if (CameraSetSaturation(hCamera, iSaturation) != CAMERA_STATUS_SUCCESS)
		std::cout << "800RMB cam CameraSetSaturation Failed!!!!!!\n";

	if (CameraSetFrameSpeed(hCamera, iFrameSpeed) != CAMERA_STATUS_SUCCESS)
		std::cout << "800RMB cam CameraSetFrameSpeed Failed!!!!!!\n";

	//sync the time
	timespec uptime;
	if (CameraRstTimeStamp(hCamera) != CAMERA_STATUS_SUCCESS)
		std::cout << "800RMB cam RstTimeStamp Failed!!!!!!\n";

	clock_gettime(CLOCK_MONOTONIC, &uptime);
	camSetTime.tv_sec = uptime.tv_sec;
	camSetTime.tv_usec = uptime.tv_nsec / 1000;

	info();

	return true;
}

FrameInfo *mvCamera::getFrame()
{
	FrameInfo *pFrame = new FrameInfo(this);
	CameraSdkStatus temp;
	if ((temp = CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000)) == CAMERA_STATUS_SUCCESS)
	{
		CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
		if (iplImage)
		{
			cvReleaseImageHeader(&iplImage);
		}
		iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, channel);
		cvSetData(iplImage, g_pRgbBuffer, sFrameInfo.iWidth * channel);
		pFrame->img = Mat(cvarrToMat(iplImage));
		Mat Iimag(cvarrToMat(iplImage)); //这里只是进行指针转换，将IplImage转换成Mat类型
		//imshow("OpenCV Demo", Iimag);
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
	fsHelper::readOrDefault(node["iGamma"], iGamma, 50);
	fsHelper::readOrDefault(node["iContrast"], iContrast, 100);
	fsHelper::readOrDefault(node["iSaturation"], iSaturation, 100);
	fsHelper::readOrDefault(node["iFrameSpeed"], iFrameSpeed, 1);
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
	   << "}";
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
	cout << tCapability.sResolutionRange.iHeightMax << endl;
	cout << tCapability.sResolutionRange.iWidthMax << endl;
	//TODO:
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