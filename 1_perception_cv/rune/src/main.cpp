#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <iostream>  
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "findRect.hpp"
#include "MnistRecognizer.h"
#include "DigitRecognizer.h"
#include "Settings.h"
#include "angleSol.h"
using namespace std;
using namespace cv;
int RiLowH = 0;
int RiHighH = 35;
int RiLowS =10;
int RiHighS = 230;
int RiLowV = 250;
int RiHighV = 255;
void preprocessHSV(Mat& image, Mat& result)
{
    Mat tempHSV;
	vector<Mat> hsvSplit;
	cvtColor(image, tempHSV, COLOR_BGR2HSV);
	split(tempHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, tempHSV);
	inRange(tempHSV, Scalar(RiLowH, RiLowS, RiLowV), Scalar(RiHighH, RiHighS, RiHighV), result);
    morphologyEx(result,result,MORPH_CLOSE,getStructuringElement(MORPH_RECT,Size(3,3)));
	imshow("HSV", result);
    return;
}
static geometry_msgs::Point ps[9];
int main(int argc, char** arg)
{
    int a = arg[1][0] - '0';
    VideoCapture cap(a);
    if(!cap.isOpened())
    {
 	 cout<< "!cap.isOpened()"<<endl;      
	 return -1;
    }
    cout<<"start"<<endl;
    Settings s("setting.xml","1.yml");
    if(!s.load())
    {
        return -1;
    }
    cout<<"load successful"<<endl;
    clock_t start, end;
    while(true)
    {
        start = clock();
        Mat img;
        cap>>img;
        imshow("input",img);
        // mnist numbers
        Mat image;
        cvtColor(img, image, CV_BGR2GRAY);
        Mat binary;
        threshold(image,binary,150,255,CV_THRESH_BINARY);
        imshow("binary",binary);
        morphologyEx( binary,  binary, MORPH_OPEN, getStructuringElement(MORPH_RECT,Size(3,3)));
        imshow("close-open", binary); 
        vector<vector<Point> > squares;
        findSquaresBinary(binary,squares);
        vector<RotatedRect> rects;
        if(checkRects(binary,squares,rects))
        {
            bool outOfImg = false;
            MnistRecognizer MR;
            for(int i = 0; i<9;i++)
            {
                cout<<"!"<<endl;
                Rect t = rects[i].boundingRect();
                if(!(0 <= t.x && 0 <= t.width && t.x + t.width <= img.cols && 0 <= t.y && 0 <= t.height && t.y + t.height <= img.rows))
                {
                    outOfImg = true;
                    break;
                }
                if (i == 4)
                {
                    AngleSolver ag;
                   
                }
                MR.mnistImgs.push_back(img(rects[i].boundingRect()));
            }
            if(outOfImg)
            {
                waitKey(20);
                continue;
            }
             if(MR.classify())
            {   
                for(int i = 1;i<=9;i++)
                {
                    putText(img,to_string(i),rects[MR.mnistLabels[i]].center, FONT_HERSHEY_SIMPLEX, 1 , Scalar(0,255,255),3);
                    AngleSolver ag;
		    double x, y,z;
                    if(ag.calculateFromImg(rects[MR.mnistLabels[i]].boundingRect(),s,x,y,z))
		    {
			ps[i].z = z;
		    	ps[i].y = y;
		    	ps[i].x = x;
		    }
                }
                imshow("a",img);
                
            }
             else
            {
                waitKey(10);
                continue;
            }
            
            
        }
        else
        {
            cout<<"not enough mnists"<<endl;
        }
        
       // imshow("after",img);
        
        waitKey(10);
    }

}
