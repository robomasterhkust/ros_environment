#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include "Settings.h"
#include "angleSol.h"
using namespace std;

void AngleSolver::rotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
	double x1 = x;//
	double y1 = y;
	double rz = thetaz * CV_PI / 180;
	outx = cos(rz) * x1 - sin(rz) * y1;
	outy = sin(rz) * x1 + cos(rz) * y1;
}

void AngleSolver::rotateByY(double x, double z, double thetay, double& outx, double& outz)
{
	double x1 = x;
	double z1 = z;
	double ry = thetay * CV_PI / 180;
	outx = cos(ry) * x1 + sin(ry) * z1;
	outz = cos(ry) * z1 - sin(ry) * x1;
}

void AngleSolver::rotateByX(double y, double z, double thetax, double& outy, double& outz)
{
	double y1 = y;
	double z1 = z;
	double rx = thetax * CV_PI / 180;
	outy = cos(rx) * y1 - sin(rx) * z1;
	outz = cos(rx) * z1 + sin(rx) * y1;
}
// rotate a point by a certain vector, return the point after rorating
Point3f AngleSolver::RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta)
{
	double r = theta * CV_PI / 180;
	double c = cos(r);
	double s = sin(r);
	double new_x = (vx*vx*(1 - c) + c) * old_x + (vx*vy*(1 - c) - vz*s) * old_y + (vx*vz*(1 - c) + vy*s) * old_z;
	double new_y = (vy*vx*(1 - c) + vz*s) * old_x + (vy*vy*(1 - c) + c) * old_y + (vy*vz*(1 - c) - vx*s) * old_z;
	double new_z = (vx*vz*(1 - c) - vy*s) * old_x + (vy*vz*(1 - c) + vx*s) * old_y + (vz*vz*(1 - c) + c) * old_z;
	return cv::Point3f(new_x, new_y, new_z);
}

void AngleSolver::setDistortionCoefficients(Settings& s)
{
	distortionCoefficients = s.cameraSetting.distortionMatrix;
}

void AngleSolver::setCameraMAtrix(Settings& s)
{
	cameraMatrix = s.cameraSetting.cameraMatrix;
}

void AngleSolver::setRealWorldTargetS(Settings& setting)
{
    int width =  setting.smallRuneSetting.width;
	int height = setting.smallRuneSetting.height;
	
	targetInWorld.push_back(Point3f(0,0,0));
	targetInWorld.push_back(Point3f(width,0,0));
	targetInWorld.push_back(Point3f(0,height, 0));
	targetInWorld.push_back(Point3f(width, height, 0));
}

bool AngleSolver::setImageTargetS(vector<cv::Point2f> input)
{
	cout << "set Image target" << endl;
	if(input.size()!=4)
	{	
		cout<<"no valid input!"<<endl;
		return false;
	}	
	Point2f vertices[4];
	int i = 0;
	for(vector<cv::Point2f>::iterator p = input.begin();p<input.end();p++)		
	{
        vertices[i] = *p;
        i++;
	}
	Point2f lu, ld, ru, rd;

	sort(vertices, vertices+4, [](const Point2f & p1, const Point2f & p2) { return p1.x < p2.x; });
	if (vertices[0].y < vertices[1].y) {
		lu = vertices[0];
		ld = vertices[1];
	}
	else {
		lu = vertices[1];
		ld = vertices[0];
	}
	if (vertices[2].y < vertices[3].y) {
		ru = vertices[2];
		rd = vertices[3];
	}
	else {
		ru = vertices[3];
		rd = vertices[2];
	}

	targetInImage.clear();
	targetInImage.push_back(lu);
	targetInImage.push_back(ru);
	targetInImage.push_back(ld);
	targetInImage.push_back(rd);
	//circle(img,lu,3,Scalar(255,0,0),3);
    //circle(img,ru,3,Scalar(255,255,0),3);
	//circle(img,ld,3,Scalar(255,0,255),3);
	//circle(img,rd,3,Scalar(0,255,0),3);
	return true;

}

void AngleSolver::getRotation_Translation_Matrix()
{
	solvePnP(targetInWorld, targetInImage, cameraMatrix, distortionCoefficients, rotationMatrix, translationMatrix, false, CV_P3P);
	//solvePnP(targetInWorld,targetInImage, cameraMatrix, distortionCoefficients,rotationMatrix,transitionMatrix, false, CV_ITERATIVE);//
	//solvePnP(targetInWorld,targetInImage, cameraMatrix, distortionCoefficients,rotationMatrix,transitionMatrix, false, CV_EPNP);//
}

void AngleSolver::getPositionInfo(double& x, double& y, double& z)
{
	double rm[9];
	cv::Mat rotM(3, 3, CV_64FC1, rm);
	Rodrigues(rotationMatrix, rotM);
	double r11 = rotM.ptr<double>(0)[0];
	double r12 = rotM.ptr<double>(0)[1];
	double r13 = rotM.ptr<double>(0)[2];
	double r21 = rotM.ptr<double>(1)[0];
	double r22 = rotM.ptr<double>(1)[1];
	double r23 = rotM.ptr<double>(1)[2];
	double r31 = rotM.ptr<double>(2)[0];
	double r32 = rotM.ptr<double>(2)[1];
	double r33 = rotM.ptr<double>(2)[2];
	this->thetaZ = atan2(r21, r11) / CV_PI * 180;
	this->thetaY = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
	this->thetaX = atan2(r32, r33) / CV_PI * 180;
	// the coordinate of rune in camera
	this->tz = translationMatrix.ptr<double>(0)[2];
	this->ty = translationMatrix.ptr<double>(0)[1];
	this->tx = translationMatrix.ptr<double>(0)[0];
	// the cooradinate of camera in world ,not useful
	double X = tx, Y = ty, Z = tz;
	rotateByZ(X, Y, -1 * thetaZ, X, Y);
	rotateByY(X, Z, -1 * thetaY, X, Z);
	rotateByX(Y, Z, -1 * thetaX, Y, Z);
	Rx = -X;
	Ry = -Y;
	Rz = -Z;
	//
	x = tx;
	y = tz;
	z = ty;
	x += sx;
	y += sy;
	z += sz;
	return;
}

void AngleSolver::setShooterParameter(const Settings setting)//get speed and angles from ros
{
	// need to know shooterAngleYaw, shooterAnglePitch,speed
	speed = setting.shooterSetting.speed;
	sx = setting.shooterSetting.Xoffset;
	sy = setting.shooterSetting.Yoffset;
	sz = setting.shooterSetting.Zoffset;
}

void AngleSolver::getShooterAngle()
{
	// TODO
}

void AngleSolver:: sendAns(Mat& img)//send angles by ros
{
	cout << "tx: " << this->tx << endl << "ty: " << this->ty << endl << "tz: " << this->tz << endl << "------" << endl;
    cout << "Rx: " << this->Rx << endl << "Ry: " << this->Ry << endl << "Rz: " << this->Rz << endl << "-----------------" << endl;
	int d = sqrt(tx*tx+ty*ty+tz*tz); 
	cout<<"distance: "<<d<<endl;
	putText(img,to_string(d),Point(100,100),FONT_HERSHEY_SIMPLEX, 1 , Scalar(0,0,255),3);
	
}
 
bool AngleSolver::calculateFromImg(Rect t,  Settings& s,double& x, double& y, double& z)
{
	setDistortionCoefficients(s);
	setCameraMAtrix(s);
	setRealWorldTargetS(s);
    vector<Point2f> input;
    input.push_back(Point2f(t.x,t.y));
    input.push_back(Point2f(t.x,t.y+t.height));
    input.push_back(Point2f(t.x+t.width,t.y));
    input.push_back(Point2f(t.x+t.width,t.y+t.height));
	if(!setImageTargetS(input))
	{
        cout<< "setImageTarget gg " <<endl;
		return false;
    }
    else
    {
		getRotation_Translation_Matrix();
		setShooterParameter(s);
	    getPositionInfo(x,y,z);
		return true;
    }
}
