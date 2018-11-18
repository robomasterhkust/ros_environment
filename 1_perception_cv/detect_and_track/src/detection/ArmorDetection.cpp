#include "defines.hpp"
#include "ArmorDetection.hpp"
#include "Settings.hpp"
#include "ros/ros.h"
#include "main.hpp"

const Scalar redLightDrawColor = Scalar(0, 255, 255);
const Scalar blueLightDrawColor = Scalar(255, 255, 0);
const Scalar redArmorDrawColor = Scalar(0, 0, 255);
const Scalar blueArmorDrawColor = Scalar(255, 0, 0);
const Scalar lightGPDrawColor = Scalar(255, 255, 255);

Light::Light(RotatedRect &_rect, vector<cv::Point> &contour)
    : rect(_rect)
{
    //LightFinder::ConvertRRectAngle2Normal(rect);

    Point2f d(-sin(rect.angle / 180.0 * CV_PI),
              cos(rect.angle / 180.0 * CV_PI));
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

float Light::length() const
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
    float d = min(min(norm(toolBody.vertex[0] - vertex[0]),
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

Armor::Armor(const Vec3f &rotation,
             const Vec3f &translation,
             const cv::Matx33d &translationCov,
             vector<Point2f> vertices,
             bool isBig,
             bool isBlue)
    : rotation(rotation),
      translation(translation),
      vertices(vertices),
      isBlue(isBlue),
      isBig(isBig),
      translationCov(translationCov){};

LightStorage::~LightStorage(){};
ArmorStorage::~ArmorStorage(){};

void LightStorage::drawLights()
{
    for (int i = 0; i < lightsB.size(); i++)
    {
        // line(this->img, lightsB[i].vertex[0], lightsB[i].vertex[1], blueLightDrawColor, 2);
        putText(this->img, to_string(i), lightsB[i].rect.center, 0, 0.5, blueLightDrawColor);
    }

    for (int i = 0; i < lightsR.size(); i++)
    {
        // line(this->img, lightsR[i].vertex[0], lightsR[i].vertex[1], blueLightDrawColor, 2);
        putText(this->img, to_string(i), lightsR[i].rect.center, 0, 0.5, redLightDrawColor);
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

void ArmorStorage::printArmors()
{
    for (int i = 0; i < armors.size(); i++)
    {
        cout << "Armor: " << armors[i].rotation << armors[i].translation << endl;
    }
};

void ArmorStorage::drawArmors()
{
    for (auto armor : armors)
    {
        if (armor.isBlue)
        {
            circle(this->preprocessedImgB, (armor.vertices[0] + armor.vertices[1] + armor.vertices[2] + armor.vertices[3]) / 4, 5, blueArmorDrawColor, 3);
            // line(this->preprocessedImgB, armor.vertices[0], armor.vertices[1], blueArmorDrawColor);
            // line(this->preprocessedImgB, armor.vertices[0], armor.vertices[2], blueArmorDrawColor);
            // line(this->preprocessedImgB, armor.vertices[3], armor.vertices[1], blueArmorDrawColor);
            // line(this->preprocessedImgB, armor.vertices[3], armor.vertices[2], blueArmorDrawColor);
        }
        else
        {
            circle(this->preprocessedImgB, (armor.vertices[0] + armor.vertices[1] + armor.vertices[2] + armor.vertices[3]) / 4, 5, redArmorDrawColor, 3);
            // line(this->preprocessedImgR, armor.vertices[0], armor.vertices[1], redArmorDrawColor);
            // line(this->preprocessedImgR, armor.vertices[0], armor.vertices[2], redArmorDrawColor);
            // line(this->preprocessedImgR, armor.vertices[3], armor.vertices[1], redArmorDrawColor);
            // line(this->preprocessedImgR, armor.vertices[3], armor.vertices[2], redArmorDrawColor);
        }
    }
};

LightStorage *LightFinder::findLight(FrameInfo *frame)
{
    LightStorage *result = new LightStorage(Mat(), Mat(), frame);
    //preprocessing start

    //TODO: improved feature extraction
    Mat tempHSV;

    if (frame->sourceCamPtr->lightFilterSetting.UseHSV != 0)
    {
        cvtColor(frame->img, tempHSV, CV_BGR2HSV);

        inRange(tempHSV,
                frame->sourceCamPtr->lightFilterSetting.HSVMinBlue,
                frame->sourceCamPtr->lightFilterSetting.HSVMaxBlue,
                result->preprocessedImgB);

        inRange(tempHSV,
                frame->sourceCamPtr->lightFilterSetting.HSVMinRed,
                frame->sourceCamPtr->lightFilterSetting.HSVMaxRed,
                result->preprocessedImgR);
    }
    else
    {

        inRange(frame->img,
                frame->sourceCamPtr->lightFilterSetting.BGRMinBlue,
                frame->sourceCamPtr->lightFilterSetting.BGRMaxBlue,
                result->preprocessedImgB);

        inRange(frame->img,
                frame->sourceCamPtr->lightFilterSetting.BGRMinRed,
                frame->sourceCamPtr->lightFilterSetting.BGRMaxRed,
                result->preprocessedImgR);
    }

    // result->preprocessedImgB = tempB;
    // result->preprocessedImgR = tempR;

    //TODO: resolution dependent element size

    //this open operation is for eliminating small noise
    // Mat ele = getStructuringElement(MORPH_RECT, Size(3, 3));
    // morphologyEx(result->preprocessedImgB, result->preprocessedImgB, MORPH_OPEN, ele);
    // morphologyEx(result->preprocessedImgR, result->preprocessedImgR, MORPH_OPEN, ele);

    Mat ele2 = getStructuringElement(MORPH_RECT, Size(frame->sourceCamPtr->lightFilterSetting.morphoRadius * 2 + 1, frame->sourceCamPtr->lightFilterSetting.morphoRadius * 2 + 1));
    morphologyEx(result->preprocessedImgR,
                 result->preprocessedImgR,
                 MORPH_CLOSE, ele2);
    morphologyEx(result->preprocessedImgB,
                 result->preprocessedImgB,
                 MORPH_CLOSE, ele2);
    result->preprocessedImgOR = result->preprocessedImgR | result->preprocessedImgB;

    //preprocessing end
    vector<vector<Point>> white_contours_crude;

    vector<RotatedRect> light_rect;
    vector<vector<Point>> light_contours;
    vector<float> contour_B_area;
    vector<float> contour_R_area;

    vector<vector<Point>> Bcontours;
    vector<vector<Point>> Rcontours;
    vector<Vec4i> hierarchy;

    findContours(result->preprocessedImgOR, white_contours_crude, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    contour_B_area.reserve(white_contours_crude.size());
    contour_R_area.reserve(white_contours_crude.size());
    light_rect.reserve(white_contours_crude.size());

    for (int i = 0; i < white_contours_crude.size(); i++)
    {

        RotatedRect tempRRect = minAreaRect(Mat(white_contours_crude[i]));
        ConvertRRectAngle2Normal(tempRRect);
        if (testAspectRatio(tempRRect) &&
            testArea(tempRRect) &&
            testTilt(tempRRect))
        {
            light_rect.push_back(tempRRect);
            light_contours.push_back(white_contours_crude[i]);
        }
        // contour_B_area.push_back(0);
        // contour_R_area.push_back(0);
    }

    findContours(result->preprocessedImgB, Bcontours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // if (detectionNodeShared::settings.Debug)
    //     drawContours(result->img, Bcontours, -1, (255, 0, 0), 3);

    findContours(result->preprocessedImgR, Rcontours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // if (detectionNodeShared::settings.Debug)
    //     drawContours(result->img, Rcontours, -1, (0, 0, 255), 3);

    for (int lr = 0; lr < light_rect.size(); lr++)
    {

        Point2f corners[4];
        light_rect[lr].points(corners);
        Point2f *lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
        vector<Point2f> vertice(corners, lastItemPointer);

        contour_B_area[lr] = 0;
        for (auto b : Bcontours)
        {
            RotatedRect tempRRect = minAreaRect(Mat(b));

            //Check if the point is within the rectangle.
            double indicator = pointPolygonTest(vertice, tempRRect.center, false);
            if (indicator >= 0)
            {
                contour_B_area[lr] += tempRRect.size.area();
            }
        }

        contour_R_area[lr] = 0;
        for (auto r : Rcontours)
        {
            RotatedRect tempRRect = minAreaRect(Mat(r));

            //Check if the point is within the rectangle.
            double indicator = pointPolygonTest(vertice, tempRRect.center, false);
            if (indicator >= 0)
            {
                contour_R_area[lr] += tempRRect.size.area();
            }
        }

        if ((contour_B_area[lr] / contour_R_area[lr]) > detectionNodeShared::settings.adSetting.red_blue_classificatino_ratio)
        {
            result->lightsB.push_back(Light(light_rect[lr], light_contours[lr]));
        }
        else if ((contour_R_area[lr] / contour_B_area[lr]) > detectionNodeShared::settings.adSetting.red_blue_classificatino_ratio)
        {
            result->lightsR.push_back(Light(light_rect[lr], light_contours[lr]));
        }
    }

    result->joinBrokenLights();
    return result;
};

/**
 * Reimplemented for the ROS node detect_and_track
 */
LightStorage *LightFinder::findLightwithSetting(Mat image, LightFilterSetting *setting)
{
    FrameInfo *empty_frame;
    LightStorage *result = new LightStorage(Mat(), Mat(), empty_frame);

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
        if (testAspectRatio(tempRRect) &&
            testArea(tempRRect) &&
            testTilt(tempRRect))
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

        // detectionNodeShared::settings.adSetting.red_blue_classificatino_ratio = 2
        if ((contour_B_area[lr] / contour_R_area[lr]) > 2) {
            result->lightsB.push_back(Light(light_rect[lr], light_contours[lr]));
        }
        else if ((contour_R_area[lr] / contour_B_area[lr]) > 2) {
            result->lightsR.push_back(Light(light_rect[lr], light_contours[lr]));
        }
    }
    result->joinBrokenLights();
    return result;
}

bool LightFinder::testAspectRatio(const RotatedRect &light)
{
    bool temp = ((light.size.height / light.size.width) >= detectionNodeShared::settings.adSetting.light_min_aspect_ratio_);
    if (detectionNodeShared::settings.Debug && !temp)
        cout << "testAspectRatio: " << light.size.height / light.size.width << endl;
    return temp;
};

bool LightFinder::testArea(const RotatedRect &light)
{
    return (light.size.area() >= detectionNodeShared::settings.adSetting.light_min_area_);
};

bool LightFinder::testTilt(const RotatedRect &light)
{
    return (abs(light.angle) <= detectionNodeShared::settings.adSetting.light_max_tilt_);
};
void LightFinder::ConvertRRectAngle2Normal(RotatedRect &rRect)
{
    if (rRect.size.width > rRect.size.height)
    {
        rRect.angle += 90;
        swap(rRect.size.width, rRect.size.height);
    }
    return;
};
bool LightFinder::tryProcess(ConcurrentQueue<FrameInfo> &inputQ, ConcurrentQueue<LightStorage> &outputQueue)
{
    ros::Time t = ros::Time::now();
    FrameInfo *tempin;
    if (inputQ.dequeue(tempin))
    {
        ROS_INFO("LightFinder...");
        //process and get a result
        LightStorage *tempout = findLight(tempin);

        if (detectionNodeShared::settings.Debug)
        {
            tempout->drawLights();
        }
        //push the result to the output queue
        outputQueue.enqueue(tempout);
        delete tempin;

        ros::Duration deltat = ros::Time::now() - t;
        ROS_INFO("LightFinder took %f ms...", deltat.toSec() * 1000.0);
        return true;
    }
    return false;
};

bool ArmorProcessor::tryProcess(ConcurrentQueue<LightStorage> &inputQ, ConcurrentQueue<ArmorStorage> &outputQueue)
{
    ros::Time t = ros::Time::now();
    LightStorage *tempin;
    if (inputQ.dequeue(tempin))
    {
        ROS_INFO("ArmorProcessor...");

        ArmorStorage *tempout = generateArmors(tempin);
        outputQueue.enqueue(tempout);

        if (detectionNodeShared::settings.Debug)
        {
            tempout->drawArmors();
            tempout->printArmors();
        }
        delete tempin;

        ROS_INFO("ArmorProcessor took %f ms, latency: %f ms",
                 (ros::Time::now() - t).toSec() * 1000.0,
                 (ros::Time::now() - tempin->rosheader.stamp).toSec() * 1000.0);
        return true;
    }
    return false;
};

ArmorStorage *ArmorProcessor::generateArmors(LightStorage *const lights)
{
    ArmorStorage *out = new ArmorStorage(lights);
    vector<LightGp> RLightGps;
    vector<LightGp> BLightGps;

    armorGrouper(lights->lightsR, RLightGps);
    armorGrouper(lights->lightsB, BLightGps);
    armorLocator(RLightGps, false, lights->sourceCamPtr, *out);
    armorLocator(BLightGps, true, lights->sourceCamPtr, *out);

    return out;
};

void ArmorProcessor::armorGrouper(const vector<Light> &srcLights, vector<LightGp> &dstLightGps)
{
    dstLightGps.clear();
    for (vector<Light>::const_iterator j = srcLights.begin(); j != srcLights.end(); j++)
    {
        //serach through found armor groups and match with the previously found armor groups
        vector<LightGp>::iterator firstMatchArmor;
        bool matched = false;

        for (vector<LightGp>::iterator i = dstLightGps.begin(); i != dstLightGps.end();)
        {
            if (i->match(*j))
                if (matched)
                {
                    firstMatchArmor->merge(*i);
                    i = dstLightGps.erase(i);
                    continue;
                }
                else
                {
                    matched = true;
                    firstMatchArmor = i;
                    i->addLight(j.base());
                }
            i++;
        }

        //no matching armor found, create new armor object
        if (!matched)
        {
            LightGp newALGP(j.base());
            dstLightGps.push_back(newALGP);
        }
    }
}

void ArmorProcessor::armorLocator(const vector<LightGp> &ArmorLightGps,
                                  bool isBlue,
                                  const Camera *const &sourceCamPtr,
                                  ArmorStorage &result)
{
    cv::Vec3d rotationVec,
        translationVec,
        shiftedTranslationVec,
        oversizedTranslationVec;
    cv::Matx33d translationCov;
    int index = 0;
    for (vector<LightGp>::const_iterator i = ArmorLightGps.begin(); i != ArmorLightGps.end(); i++, index++)
    {
        //TODO: multiple grouped light handling
        //TODO: incomplete armor pulishing
        //TODO: ignore too close
        i->paintOnMat(result.img, index);
        if (i->lights.size() >= 2)
        {
            //idea:  find distance between lights, choose light pair having closest distance to small armor width / height * average height
            float sum = 0;
            for (auto light : i->lights)
            {
                sum += light->length();
            }
            //average lenght o lights
            float avg_length = sum / (float)i->lights.size();
            //best seperation between lights to consider them as a armor
            float target_saparation = avg_length * SMALL_ARMOR_WIDTH / ALL_ARMOR_HEIGHT;

            //find pairs of lights that is closest to ideal
            float min_diff = abs(cv::norm(i->lights[0]->rect.center - i->lights[1]->rect.center) - target_saparation);
            int min_start_id = 0;

            if (detectionNodeShared::settings.Debug)
                putText(result.preprocessedImgB, to_string(min_diff), (i->lights[0]->rect.center + i->lights[1]->rect.center) / 2, 0, 0.25, cv::Scalar(255, 255, 255, 2555));

            for (int startid = 1; startid < (i->lights.size() - 1); startid++)
            {
                float temp = abs(cv::norm(i->lights[startid]->rect.center - i->lights[startid + 1]->rect.center) - target_saparation);
                //    ROS_INFO("d sep: %f", temp);
                if (detectionNodeShared::settings.Debug)
                    putText(result.preprocessedImgB, to_string(temp), (i->lights[startid]->rect.center + i->lights[startid + 1]->rect.center) / 2, 0, 0.25, cv::Scalar(255, 255, 255, 2555));
                if (temp < min_diff)
                {
                    min_start_id = startid;
                    min_diff = temp;
                }
            }

            //find the aspect ratio of the best fit armor
            float dist = cv::norm(i->lights[min_start_id]->rect.center - i->lights[min_start_id + 1]->rect.center);
            float tempR = dist / (i->lights[min_start_id + 1]->rect.size.height + i->lights[min_start_id]->rect.size.height) * 2.0;

            bool isBig = i->getMaxRatio() > ARMOR_GROUPING_BIG_SMALL_ASPECT_RATIO_THRESHOLD;
            vector<cv::Point3f> *points = &detectionNodeShared::settings.adSetting.realArmorPoints_Big;

            if (isBig)
                points = &detectionNodeShared::settings.adSetting.realArmorPoints_Big;
            else
                points = &detectionNodeShared::settings.adSetting.realArmorPoints;

            vector<Point2f> vertices;
            i->getVertices(vertices);
            vector<Point2f> bestvertices;

            bestvertices.reserve(4);
            for (int j = min_start_id * 2; j < (min_start_id * 2 + 4); j++)
            {
                bestvertices.push_back(vertices[j]);
            }

            cv::Point2f center = bestvertices[0] + bestvertices[1] + bestvertices[2] + bestvertices[3];
            center /= 4.0;

            //solve result if oversized
            vector<Point2f> oversizedVertices;
            oversizedVertices.reserve(4);
            for (int j = 0; j < 4; j++)
            {
                oversizedVertices.push_back((bestvertices[j] - center) * (1.0 + 2 / cv::norm(bestvertices[j] - center)) + center);
            }

            sourceCamPtr->solvePNP(*points,
                                   oversizedVertices,
                                   rotationVec,
                                   oversizedTranslationVec,
                                   false,
                                   SOLVEPNP_P3P);

            //solve  result if shifted
            vector<Point2f> shiftedVertices;
            shiftedVertices.reserve(4);
            for (int j = 0; j < 4; j++)
            {
                shiftedVertices.push_back(bestvertices[j] + cv::Point2f(1, 1));
            }

            sourceCamPtr->solvePNP(*points,
                                   shiftedVertices,
                                   rotationVec,
                                   shiftedTranslationVec,
                                   false,
                                   SOLVEPNP_P3P);

            //solve best estimate
            sourceCamPtr->solvePNP(*points,
                                   bestvertices,
                                   rotationVec,
                                   translationVec,
                                   false,
                                   SOLVEPNP_P3P);

            if (detectionNodeShared::settings.Debug)
            {
                cout << "translationVec\n"
                     << translationVec << endl;
                cout << "shiftedTranslationVec\n"
                     << shiftedTranslationVec << endl;
                cout << "oversizedTranslationVec\n"
                     << oversizedTranslationVec << endl;
            }

            //solve covariance
            double dx = abs(shiftedTranslationVec[0] - translationVec[0]) + abs(oversizedTranslationVec[0] - translationVec[0]);
            double dy = abs(shiftedTranslationVec[1] - translationVec[1]) + abs(oversizedTranslationVec[1] - translationVec[1]);
            double dz = abs(shiftedTranslationVec[2] - translationVec[2]) + abs(oversizedTranslationVec[2] - translationVec[2]);
            double cov_xx = pow(dx * 2, 2) / 12.0;
            double cov_yy = pow(dy * 2, 2) / 12.0;
            double cov_zz = pow(dz * 2, 2) / 12.0;

            // double cov_xx = 0;
            // double cov_yy = 0;
            // double cov_zz = 0;

            //somewhat linearly correlated
            double cov_xy = sqrt(cov_xx * cov_yy) / 2.0;
            double cov_xz = sqrt(cov_xx * cov_zz) / 2.0;
            double cov_yz = sqrt(cov_yy * cov_zz) / 2.0;

            if (translationVec[0] < 0)
            {
                cov_xz = -cov_xz;
                if (translationVec[1] > 0)
                    //x -, y+
                    cov_xy = -cov_xy;
                else
                    //x-, y-
                    cov_yz = -cov_yz;
            }
            else if (translationVec[1] < 0)
            {
                //x+, y-
                cov_xy = -cov_xy;
                cov_yz = -cov_yz;
            }

            translationCov = Matx33d(cov_xx, cov_xy, cov_xz,
                                     cov_xy, cov_yy, cov_yz,
                                     cov_xz, cov_yz, cov_zz);
            //apply camera transformation and translation
            sourceCamPtr->rectifyCoor(translationVec, translationCov);

            result.armors.push_back(Armor(rotationVec, translationVec, translationCov, bestvertices, isBig, isBlue));
        }
    }
};

bool ArmorProcessor::testSeparation(const Light *a, const Light *b)
{
    double meanLength = 0.5 * (a->rect.size.height + b->rect.size.height);
    //cout << "meanLength " << meanLength << endl;
    double distance = norm(a->rect.center - b->rect.center);
    // cout << "distance " << distance << endl;
    return (distance / meanLength <= detectionNodeShared::settings.adSetting.armor_max_aspect_ratio_) &&
           (distance / meanLength >= detectionNodeShared::settings.adSetting.armor_min_aspect_ratio_);
};

bool ArmorProcessor::testHeading(const Light *a, const Light *b)
{
    float tiltDiff = abs(a->rect.angle - b->rect.angle);
    if (tiltDiff > 90)
    {
        tiltDiff = 180.0 - tiltDiff;
    }
    return tiltDiff <= detectionNodeShared::settings.adSetting.armor_max_tilt_diff_;
}; //check both the orientation of lights, and the angle formed between them

bool ArmorProcessor::testTilt(const Light *a, const Light *b)
{
    //angle of joining line
    double angle = atan2(a->rect.center.y - b->rect.center.y,
                         a->rect.center.x - b->rect.center.x) *
                   180.0 / CV_PI;
    //range of angle in [+180,-180], correct to [+90,-90]
    if (angle > 90.0)
        angle -= 180.0;
    else if (angle < -90.0)
        angle += 180.0;

    float d1 = abs(a->rect.angle - angle),
          d2 = abs(b->rect.angle - angle);

    if (d1 > 90.0)
    {
        d1 = 180.0 - d1;
    }
    if (d2 > 90.0)
    {
        d2 = 180.0 - d2;
    }

    return (d1 <= detectionNodeShared::settings.adSetting.armor_max_anglePos_diff_ && d2 <= detectionNodeShared::settings.adSetting.armor_max_anglePos_diff_);
};

bool ArmorProcessor::testSize(const Light *a, const Light *b)
{
    double meanLength = 0.5 * (a->rect.size.height + b->rect.size.height);
    float distance = norm(a->rect.center - b->rect.center);
    bool temp = (meanLength * distance >= detectionNodeShared::settings.adSetting.armor_min_area_);
    if (!temp && detectionNodeShared::settings.Debug)
        cout << "test Size: " << meanLength * distance << endl;
    return temp;
};

bool ArmorProcessor::testLength(const Light *a, const Light *b)
{
    return (abs((a->rect.size.height - b->rect.size.height) / (a->rect.size.height + b->rect.size.height)) <
            detectionNodeShared::settings.adSetting.armor_max_light_length_diff_proportion_);
}

ArmorProcessor::LightGp::LightGp(const Light *firstlight)
{
    lights.clear();
    relativePositions.clear();
    heading = firstlight->vertex[0] - firstlight->vertex[1];
    normal = Vec2f(-heading[1], heading[0]);
    addLight(firstlight);
};

ArmorProcessor::LightGp::~LightGp(){};

void ArmorProcessor::LightGp::coutInfo() const {};

void ArmorProcessor::LightGp::paintOnMat(Mat &img, int num) const
{
    vector<Point2f> temp;
    getVertices(temp);

    for (int i = 0; i < temp.size(); i += 2)
    {
        if (i > 0)
        {
            line(img,
                 temp[i],
                 temp[i - 2],
                 lightGPDrawColor);
            line(img,
                 temp[i + 1],
                 temp[i - 1],
                 lightGPDrawColor);
            putText(img, to_string(i), (temp[i] + temp[i - 1] + temp[i - 2] + temp[i + 1]) / 4, 0, 0.5, Scalar(255, 255, 255));
        }
        circle(img, temp[i], 3, lightGPDrawColor);
        circle(img, temp[i + 1], 3, lightGPDrawColor);
    }
};

void ArmorProcessor::LightGp::getVertices(vector<Point2f> &points) const
{
    points.clear();
    for (int i = 0; i < this->lights.size(); i++)
    {
        Vec2f temp = lights[i]->vertex[0] - lights[i]->vertex[1];

        if (temp.dot(heading) > 0)
        {
            points.push_back(lights[i]->vertex[0]);
            points.push_back(lights[i]->vertex[1]);
        }
        else
        {
            points.push_back(lights[i]->vertex[1]);
            points.push_back(lights[i]->vertex[0]);
        }
    }
};

float ArmorProcessor::LightGp::getMaxRatio() const
{
    float maxR = 0;
    if (lights.size() >= 2)
    {
        for (int i = 1; i < lights.size(); i++)
        {
            float dist = cv::norm(lights[i]->rect.center - lights[i - 1]->rect.center);
            float tempR = dist / (lights[i - 1]->rect.size.height + lights[i]->rect.size.height) * 2.0;
            if (tempR > maxR)
            {
                maxR = tempR;
            }
        }
    }
    return maxR;
};

void ArmorProcessor::LightGp::merge(LightGp &tool)
{
    vector<const Light *>::iterator i, j;
    i = lights.begin();
    for (j = tool.lights.begin(); j != tool.lights.end(); j++)
    {
        for (; i != lights.end(); i++)
            if ((*i)->rect.center.x > (*j)->rect.center.x)
                break;
        i = lights.insert(i, *j);
    }
};

void ArmorProcessor::LightGp::addLight(const Light *newlight)
{
    cout << "normal: " << normal << endl;
    cout << "center: " << newlight->rect.center << endl;
    float newPos = abs(newlight->rect.center.dot(this->normal));
    cout << "projection: " << newPos << endl;
    int i = 0;
    while (i < relativePositions.size())
    {
        if (newPos < relativePositions[i])
        {
            relativePositions.insert(relativePositions.begin() + i, newPos);
            lights.insert(lights.begin() + i, newlight);
            return;
        }
        else
        {
            i++;
        }
    }
    //the new light is the most extreme:
    lights.push_back(newlight);
    relativePositions.push_back(newPos);
};

bool ArmorProcessor::LightGp::match(const Light &light)
{
    vector<const Light *>::const_iterator i;
    for (i = lights.begin(); i != lights.end(); i++)
    {
        if (!testLength(*i, &light))
            continue;
        //cout << "length ok\n";

        if (!testTilt(*i, &light))
            continue;
        //cout << "tilt ok\n";

        if (!testSeparation(*i, &light))
            continue;
        //cout << "separation ok\n";

        if (!testHeading(*i, &light))
            continue;
        //cout << "orientation ok\n";

        if (!testSize(*i, &light))
            continue;
        //cout << "size ok\n";

        return true;
    }
    return false;
};
