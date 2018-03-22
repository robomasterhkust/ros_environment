#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <thread>
#include <cmath>
using namespace std;
using namespace cv;
typedef vector<Point> Contour;
int minCountourSize = 100;

class Pair
{
    public:
    bool isCompeleted;
    RotatedRect* memberOne;
    RotatedRect* memberTwo;
    double size;
    enum ID {HERO,INFANTRY,NONE} id;
    Point center;

    Pair()
    {
        isCompeleted = false;
        memberOne = NULL;
        memberTwo = NULL;
        size = 0;
        id = NONE;
    }
    
    ~Pair()
    {
        delete memberOne;
        delete memberTwo;
    }
    
    void addRect(Contour& contour)
    {
        if(memberOne == NULL)
        {
            memberOne = new RotatedRect;
            *memberOne = minAreaRect(contour);
            return;
        }
    }

    bool match(Contour& contour)//如果匹配的不是最合适的边怎么办
    {
        if(isCompeleted)
        return false;
        if(memberOne == NULL)
        return false;

        float angleOne = memberOne -> angle;
        RotatedRect* temp = new RotatedRect;
        *temp = minAreaRect(contour);
        float angleTwo = temp -> angle;
        float dif = angleOne - angleTwo;
        if(dif*dif > 100 )
        {
            delete temp;
            return false;
        }
        if(dif*dif <= 100)
        {
            memberTwo = temp;
            isCompeleted = true;
            id = findID();
            size = findSize();
            center = findCenter();
            return true;
        }
    }

    ID findID()
    {
        float distance = sqrt((memberOne->center.x - memberTwo->center.x)*(memberOne->center.x - memberTwo->center.x)+(memberOne->center.x - memberTwo->center.x)*(memberOne->center.y - memberTwo->center.y));
        float r = distance/ ((memberOne->size.height+memberTwo->size.height)/2);
        if( r > 10 )
        return NONE;
        else if(r > 3)
        return HERO;
        else
        return INFANTRY;
    }

    Point findCenter()
    {
       Point center;
       center.x = (memberOne->center.x - memberTwo->center.x)/2;
       center.y = (memberOne->center.y - memberTwo->center.y)/2;
       return center;
    }
    
    float findSize()
    {
         float distance = sqrt((memberOne->center.x - memberTwo->center.x)*(memberOne->center.x - memberTwo->center.x)+(memberOne->center.x - memberTwo->center.x)*(memberOne->center.y - memberTwo->center.y));
         float h = (memberOne->size.height+ memberTwo->size.height)/2;
         return distance*h;
    }
};