//
// Created by beck on 21/11/18.
//

#include "armor_detect.h"

/**
 * Light constructor and private functions
 */
Light::Light(RotatedRect &_rect, vector<cv::Point> &contour)
        : rect(_rect)
{
    Point2f d(-sinf(rect.angle / 180.0 * CV_PI),
              cosf(rect.angle / 180.0 * CV_PI));
    float max = contour[0].dot(d);
    int maxID = 0;
    int minID = 0;
    float min = contour[0].dot(d);

    for (int i = 0; i < contour.size(); i++)
    {
        float temp = contour[i].dot(d);
        if (temp > max)
        {
            max = temp;
            maxID = i;
        }
        else if (temp < min)
        {
            min = temp;
            minID = i;
        }
    }

    //find mean point
    Point maxPtSum = contour[maxID];
    int maxPtCount = 1;
    Point minPtSum = contour[minID];
    int minPtCount = 1;

    int i = maxID + 1;
    while (i < contour.size())
    {
        if ((max - contour[i].dot(d)) < rect.size.width / 2)
        {
            maxPtSum += contour[i];
        }
        else
        {
            break;
        }
        i++;
    }
    maxPtCount += i - maxID - 1;

    i = maxID - 1;
    while (i >= 0)
    {
        if ((max - contour[i].dot(d)) < rect.size.width / 2)
        {
            maxPtSum += contour[i];
        }
        else
            break;
        i--;
    }
    maxPtCount += maxID - 1 - i;

    i = minID + 1;
    while (i < contour.size())
    {
        if ((contour[i].dot(d) - min) < rect.size.width / 2)
        {
            minPtSum += contour[i];
        }
        else
        {
            break;
        }
        i++;
    }
    minPtCount += i - minID - 1;

    i = minID - 1;
    while (i >= 0)
    {
        if ((contour[i].dot(d) - min) < rect.size.width / 2)
        {
            minPtSum += contour[i];
        }
        else
            break;
        i--;
    }
    minPtCount += minID - 1 - i;

    this->vertex[1] = maxPtSum / maxPtCount;
    this->vertex[0] = minPtSum / minPtCount;
};

double Light::length() const
{
    return norm(vertex[0] - vertex[1]);
};

bool Light::merge(const Light &toolBody)
{

    //determine colinearity by furtest projcetion on normal axis by pixel value
    //and closest distance between the 4 vertice relative to length

    // axis:
    Vec2f parallel = (vertex[0] - vertex[1]);
    Vec2f normal;
    normal[0] = -parallel[1];
    normal[1] = parallel[0];
    //normalized vectors
    normal /= sqrt(pow(normal[0], 2) + pow(normal[1], 2));

    //position of this light in normal axis unit
    float nOrigin = vertex[1].dot(normal);

    //check projection of tool on normal axis
    //projection fall out of range if maximum distance > k*this light's length
    float nd1 = toolBody.vertex[0].dot(normal) - nOrigin;
    if (nd1 > LIGHT_MERGE_MAX_PERPENDICULAR_DISTANCE || nd1 < -LIGHT_MERGE_MAX_PERPENDICULAR_DISTANCE)
    {
        return false;
    }
    //cout << "nd1: " << nd1 << endl;
    float nd2 = toolBody.vertex[1].dot(normal) - nOrigin;
    if (nd2 > LIGHT_MERGE_MAX_PERPENDICULAR_DISTANCE || nd2 < -LIGHT_MERGE_MAX_PERPENDICULAR_DISTANCE)
    {
        return false;
    }
    //cout << "nd2: " << nd2 << endl;

    //check closest distance between the 4 vertice
    double d = min(min(norm(toolBody.vertex[0] - vertex[0]),
                      norm(toolBody.vertex[0] - vertex[1])),
                  min(norm(toolBody.vertex[1] - vertex[0]),
                      norm(toolBody.vertex[1] - vertex[1])));
    //cout << "d: " << d << endl;
    if (d > max(LIGHT_MERGE_MAX_DISTANCE_MULT * this->length(), LIGHT_MERGE_MAX_DISTANCE_MULT * toolBody.length()))
    {
        return false;
    }

    cv::Point2f temp[8];
    this->rect.points(temp);
    toolBody.rect.points(&temp[4]);
    vector<Point2f> tempvc;
    tempvc.assign(temp, temp + 8);
    rect = cv::minAreaRect(tempvc);

    vector<Point> tempVertice;
    tempVertice.push_back(vertex[0]);
    tempVertice.push_back(vertex[1]);
    tempVertice.push_back(toolBody.vertex[0]);
    tempVertice.push_back(toolBody.vertex[1]);

    *this = Light(rect, tempVertice);
    return true;
};


/**
 * LightStorage class private function
 */
void LightStorage::drawLights(Mat &img)
{
    for (int i = 0; i < lightsB.size(); i++)
    {
        // line(this->img, lightsB[i].vertex[0], lightsB[i].vertex[1], blueLightDrawColor, 2);
        cv::putText(img, to_string(i), lightsB[i].rect.center, 0, 0.5, blueLightDrawColor);
    }

    for (int i = 0; i < lightsR.size(); i++)
    {
        // line(this->img, lightsR[i].vertex[0], lightsR[i].vertex[1], blueLightDrawColor, 2);
        cv::putText(img, to_string(i), lightsR[i].rect.center, 0, 0.5, redLightDrawColor);
    }
};

void LightStorage::joinBrokenLights()
{
    //algo: select one light, get a orientation in radian, find all other lights that are colinear
    //colinearity tested by difference between orientations and line joining them
    //check tool bodies first by their vertice coordinates, these should be in a short range relative to the main body's own x and y span
    //once merged, use the tool light as body to serach for further merging
    for (int body_index = 0; body_index < lightsB.size(); body_index++)
    {
        for (int tool_index = body_index + 1; tool_index < lightsB.size(); tool_index++)
        {
            if (lightsB[body_index].merge(lightsB[tool_index]))
            {
                cout << "Merged a Light!\n";
                lightsB.erase(lightsB.begin() + tool_index);
                //check the previous ones again
                tool_index = body_index + 1;
            }
        }
    }

    for (int body_index = 0; body_index < lightsR.size(); body_index++)
    {
        for (int tool_index = body_index + 1; tool_index < lightsR.size(); tool_index++)
        {
            if (lightsR[body_index].merge(lightsR[tool_index]))
            {
                cout << "Merged a Light!\n";
                lightsR.erase(lightsR.begin() + tool_index);
                //check the previous ones again
                tool_index = body_index + 1;
            }
        }
    }
    //printf("\t found %ld blue and %ld red lights\n", lightsB.size(), lightsR.size());
};


/**
 * Reimplemented for the ROS node detect_and_track
 */
LightStorage *LightFinder::findLight(Mat image, LightFilterSetting *setting, Settings *armor_setting)
{
    LightStorage *result = new LightStorage(Mat(), Mat());

    // preprocessing start
    Mat tempHSV;

    if (setting->UseHSV != 0) {
        cvtColor(image, tempHSV, CV_BGR2HSV);

        inRange(tempHSV, setting->HSVMinBlue, setting->HSVMaxBlue, result->preprocessedImgB);
        inRange(tempHSV, setting->HSVMinRed,  setting->HSVMaxRed,  result->preprocessedImgR);
    }
    else {
        inRange(image, setting->BGRMinBlue, setting->BGRMaxBlue, result->preprocessedImgB);
        inRange(image, setting->BGRMinRed,  setting->BGRMaxRed,  result->preprocessedImgR);
    }

    Mat kernel = getStructuringElement(MORPH_RECT,
                                       Size(setting->morphoRadius * 2 + 1, setting->morphoRadius * 2 + 1));
    morphologyEx(result->preprocessedImgB, result->preprocessedImgB, MORPH_CLOSE, kernel);
    morphologyEx(result->preprocessedImgR, result->preprocessedImgR, MORPH_CLOSE, kernel);
    result->preprocessedImgOR = result->preprocessedImgR | result->preprocessedImgB;
    // preprocessing end

    vector<vector<Point>> white_contours;
    vector<vector<Point>> light_contours;
    vector<vector<Point>> blue_contours;
    vector<vector<Point>> red_contours;
    vector<RotatedRect> light_rect;
    vector<float> contour_B_area;
    vector<float> contour_R_area;
    vector<Vec4i> hierarchy;

    findContours(result->preprocessedImgOR, white_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    findContours(result->preprocessedImgB, blue_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    findContours(result->preprocessedImgR, red_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    contour_B_area.reserve(white_contours.size());
    contour_R_area.reserve(white_contours.size());
    light_rect.reserve(white_contours.size());

    for (size_t i = 0; i < white_contours.size(); i++) {
        RotatedRect tempRRect = minAreaRect(Mat(white_contours[i]));
        ConvertRRectAngle2Normal(tempRRect);
        if (testAspectRatio(tempRRect, armor_setting) &&
            testArea(tempRRect, armor_setting) &&
            testTilt(tempRRect, armor_setting))
        {
            light_rect.push_back(tempRRect);
            light_contours.push_back(white_contours[i]);
        }
    }

    for (size_t lr = 0; lr < light_rect.size(); lr++) {
        Point2f corners[4];
        light_rect[lr].points(corners);
        Point2f *lastItemPointer = (corners + sizeof(corners) / sizeof(corners[0]));
        vector<Point2f> vertice(corners, lastItemPointer);

        contour_B_area[lr] = 0;
        contour_R_area[lr] = 0;

        for (auto b : blue_contours) {
            RotatedRect tempRRect = minAreaRect(Mat(b));

            // check if the point is within the rectangle
            if (pointPolygonTest(vertice, tempRRect.center, false) >= 0) {
                contour_B_area[lr] += tempRRect.size.area();
            }
        }

        for (auto r : red_contours) {
            RotatedRect tempRRect = minAreaRect(Mat(r));

            // check if the point is within the rectangle
            if (pointPolygonTest(vertice, tempRRect.center, false) >= 0) {
                contour_R_area[lr] += tempRRect.size.area();
            }
        }

        double ratio = armor_setting->adSetting.red_blue_classificatino_ratio;
        if ((contour_B_area[lr] / contour_R_area[lr]) > ratio) {
            result->lightsB.push_back(Light(light_rect[lr], light_contours[lr]));
        }
        else if ((contour_R_area[lr] / contour_B_area[lr]) > ratio) {
            result->lightsR.push_back(Light(light_rect[lr], light_contours[lr]));
        }
    }
    result->joinBrokenLights();
    return result;
}


void LightFinder::ConvertRRectAngle2Normal(RotatedRect &rRect)
{
    if (rRect.size.width > rRect.size.height)
    {
        rRect.angle += 90;
        swap(rRect.size.width, rRect.size.height);
    }
    return;
};

bool LightFinder::testAspectRatio(const RotatedRect &light, Settings *armor_setting)
{
    bool temp = ((light.size.height / light.size.width) >= armor_setting->adSetting.light_min_aspect_ratio_);
    if (armor_setting->Debug && !temp)
        cout << "testAspectRatio: " << light.size.height / light.size.width << endl;
    return temp;
};

bool LightFinder::testArea(const RotatedRect &light, Settings *armor_setting)
{
    return (light.size.area() >= armor_setting->adSetting.light_min_area_);
};

bool LightFinder::testTilt(const RotatedRect &light, Settings *armor_setting)
{
    return (abs(light.angle) <= armor_setting->adSetting.light_max_tilt_);
};
