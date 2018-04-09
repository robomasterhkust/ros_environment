
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <math.h>
#include <string.h>

using namespace cv;
using namespace std;


int thresh = 50, N = 5;

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle(Point pt1, Point pt2, Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
void findSquares(const Mat& image, vector<vector<Point> >& squares)
{
    squares.clear();
   
    // blur will enhance edge detection
    Mat timg(image);
    cv::medianBlur(image, timg, 9);
    Mat gray0(timg.size(), CV_8U), gray;
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for (int c = 0; c < 3; c++)
    {
        int ch[] = { c, 0 };
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for (int l = 0; l < N; l++)
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if (l == 0)
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 5, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1, -1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l + 1) * 255 / N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // test each contour
            for (size_t i = 0; i < contours.size(); i++)
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if (approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx)))
                {
                    double maxCosine = 0;

                    for (int j = 2; j < 5; j++)
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if (maxCosine < 0.3)
                        squares.push_back(approx);
                }
            }
        }
    }
}

void findSquaresBinary(const Mat& image,  vector<vector<Point> >& squares)
{
    
    vector<vector<Point> > contours;
    findContours(image, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    vector<Point> approx;

            // test each contour
            for (size_t i = 0; i < contours.size(); i++)
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if (approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx)))
                {
                    double maxCosine = 0;

                    for (int j = 2; j < 5; j++)
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if (maxCosine < 0.3)
                        squares.push_back(approx);
                }
            }
}

// the function draws all the squares in the image
static void drawSquares(Mat& image, const vector<vector<Point> >& squares)
{
    for (size_t i = 0; i < squares.size(); i++)
    {
        const Point* p = &squares[i][0];

        int n = (int)squares[i].size();
        //dont detect the border
        if (p->x > 3 && p->y > 3)
            polylines(image, &p, &n, 1, true, Scalar(0, 255, 0), 3, CV_AA);
        /*
        RotatedRect minRect = minAreaRect(squares[i]);
        int h = minRect.size.height;
        int w = minRect.size.width;
        string text = "size : " + to_string(w) + " , "+ to_string(h);
        putText(image,text,minRect.center, FONT_HERSHEY_SIMPLEX, 1 , Scalar(255,255,255));
        */

    }
    imshow("Square Detection Demo", image);
}

bool ascendingY(const RotatedRect& a,const RotatedRect& b) {return (a.center.y<b.center.y);}
bool ascendingX(const RotatedRect& a,const RotatedRect& b) {return (a.center.x<b.center.x);}
bool descendingY(const RotatedRect& a,const RotatedRect& b) {return (a.center.y>b.center.y);}
bool descendingX(const RotatedRect& a,const RotatedRect& b) {return (a.center.x>b.center.x);}

struct RectWithDist
{
    RotatedRect r;
    float d;
};

bool checkRects(Mat& img, vector<vector<Point> >& squares,vector<RotatedRect>& rects)
{
    // too few squares
    if(squares.size()<9) 
    {
        cout<<"squares.size()<9"<<endl;
        return false;
    }
    rects.clear();
    // filtering rects by size & height/width ratio
    cout<< "_________________________"<<endl;
    for(int i = 0; i < squares.size();i++) 
    {
        RotatedRect minRect = minAreaRect(squares[i]);
        cout<< "size of "<< i << "  "<<minRect.size.width << " , "<<minRect.size.height<<endl;
        if(minRect.size.height<70||minRect.size.width<35)//||minRect.size.width>minRect.size.height*0.8)
        {
            continue;
        }
        else
        {
            rects.push_back(minRect);
        }
    }
    cout<< "_________________________"<<endl;

    if(rects.size()<9)
    {
        cout<<"------rects.size()<9"<<endl;
    }
    sort(rects.begin(),rects.end(),ascendingX);
    vector<RotatedRect>::iterator p = rects.begin();
   
    // eliminate the duplicated rects recognized 
    for(;p<rects.end();)
    {
        if(abs(p->center.x - (p+1)->center.x)<5 && abs(p->center.y - (p+1)->center.y)<5)
        {
            p = rects.erase(p);                                                       
        }
        else
        {
            p++;
        }
    }

    // too few rects
    if(rects.size()<9)
    {
        cout<<"rects.size()<9"<<endl;
    }
    sort(rects.begin(),rects.end(),[](RotatedRect& a, RotatedRect& b){return a.size.area() > b.size.area();});
    double Area= rects[3].size.area()*0.8;
    // eliminate too small rects
    for(vector<RotatedRect>::iterator p = rects.begin(); p < rects.end();)
    {
        if(p->size.area() < Area)
        {
            p = rects.erase(p);
        }
        else
        {
            p++;
        }
    }
    // pick 9 sudoku from them
    if (rects.size() > 9) 
	{
        float **dist_map = new float *[rects.size()];
        int rsize = rects.size();
		for (int i = 0; i < rsize; i++)
		{
			dist_map[i] = new float[rsize];
			for (int j = 0; j < rsize; j++)
			{
				dist_map[i][j] = 0;
			}
		}
		// calculate distance of each cell center
		for (int i = 0; i < rsize; ++i)
		{
			for (int j = i + 1; j < rsize; ++j)
			{
				int dx = rects[i].center.x - rects[j].center.x;
                int dy = rects[i].center.y - rects[j].center.y; 
                double d = sqrt(dx*dx + dy*dy);   
				dist_map[i][j] = d;
				dist_map[j][i] = d;
			}
		}
        // choose the minimun distance cell as center cell
		int center_idx = 0;
		float min_dist = 100000000;
		for (int i = 0; i < rsize; ++i)
		{
			float cur_d = 0;
			for (int j = 0; j < rsize; ++j)
			{
				cur_d += dist_map[i][j];
			}
			if (cur_d < min_dist)
			{
				min_dist = cur_d;
				center_idx = i;
			}
		}
        for (int i = 0; i < rsize; i++)
		{
			delete[] dist_map[i];
		}
		delete[] dist_map;
        // find the distance between other rect and center rect
        rectangle(img,rects[center_idx].boundingRect(),Scalar(0,255,255),5);
        imshow("center",img);
        vector<RectWithDist> RWD;
        for(int i=0; i<rsize; i++)
        {
            RectWithDist temp;
            int dx = rects[i].center.x - rects[center_idx].center.x;
            int dy = rects[i].center.y - rects[center_idx].center.y;
            temp.d = sqrt( dx*dx + dy*dy);
            temp.r = rects[i];
            RWD.push_back(temp);
        }

		// sort distance between each cell and the center cell and choose the nearest 9 cell as suduku
		std::sort(RWD.begin(),RWD.end(), [](RectWithDist &rwd1, RectWithDist &rwd2) { return rwd1.d < rwd2.d; });
        rects.clear();
        if(RWD.size()<9)
        {
            cout<<"RWD.size()<9"<<endl;
            return false;
        }
        int count = 0;
        int i = 0;
        while(count < 9 && i <RWD.size())
        {
            if(RWD[i].r.size.area() < RWD[0].r.size.area()*0.8)
            {
                i++;
                continue;
            }
            rects.push_back(RWD[i].r);
            i++;
            count++;
        }

	}
    // possibly what we want
    if(rects.size()==9)
    {
        /*
               arrange the rects, hopefully to have:
               0 1 2
               3 4 5
               6 7 8
        */
        sort(rects.begin(),rects.end(),ascendingY);                
        sort(rects.begin(),rects.begin()+3,ascendingX);          
        sort(rects.begin()+3,rects.begin()+6,ascendingX);
        sort(rects.begin()+6,rects.begin()+9,ascendingX);
        cout<<"1 , 2 , 3"<<endl;
        cout<<"4 , 5 , 6"<<endl;
        cout<<"7 , 8 , 9"<<endl;

        return true;
    }
    return false;
    
}