#include "defines.hpp"
#include "ArmorDetection.hpp"
#include "Settings.hpp"
#include "StopWatch.hpp"
extern Settings settings;

const Scalar lightDrawColor = Scalar(255, 0, 0);
const Scalar armorDrawColor = Scalar(0, 0, 255);

StopWatch lss("lightfinder");
StopWatch ads("adprocessor");

Light::Light(RotatedRect &_rect)
{
    rect = _rect;
    Point2f d(-rect.size.height / 2.0 * sin(rect.angle / 180.0 * CV_PI),
              rect.size.height / 2.0 * cos(rect.angle / 180.0 * CV_PI));
    this->vertex[0] = rect.center - d;
    this->vertex[1] = rect.center + d;
};

Armor::Armor(const Vec3f &rotation,
             const Vec3f &translation,
             vector<Point2f> vertices)
    : rotation(rotation),
      translation(translation),
      vertices(vertices){};

void LightStorage::draw()
{
    Mat temp = Mat::zeros(preprocessedImg.size(), CV_8UC3);
    for (int i = 0; i < lights.size(); i++)
    {
        line(temp, lights[i].vertex[0], lights[i].vertex[1], lightDrawColor, 3);
    }
    imshow("lightfinder", temp);
    imshow("preprocessedImg", preprocessedImg);
};

void ArmorStorage::printArmors()
{
    for (int i = 0; i < armors.size(); i++)
    {
        cout << armors[i].rotation << armors[i].translation << endl;
    }
};

void ArmorStorage::draw()
{
    Mat img = Mat::zeros(imgSize, CV_8UC3);
    for (int i = 0; i < armors.size(); i++)
    {
        line(img, armors[i].vertices[0], armors[i].vertices[1], armorDrawColor, 3);
        line(img, armors[i].vertices[0], armors[i].vertices[2], armorDrawColor, 3);
        line(img, armors[i].vertices[3], armors[i].vertices[1], armorDrawColor, 3);
        line(img, armors[i].vertices[3], armors[i].vertices[2], armorDrawColor, 3);
    }
    imshow("armor", img);
    timespec uptime;
    clock_gettime(CLOCK_MONOTONIC, &uptime);
    double dt = (double)uptime.tv_sec + uptime.tv_nsec / 1000000000.0 - capTime.tv_sec - capTime.tv_usec / 1000000.0;
    cout << "latercy: " << dt << "s\n";
};

LightFinder::LightFinder(ConcurrentQueue<FrameInfo> *_inputQ)
    : inputQ(_inputQ){};

void LightFinder::addOutputQueue(ConcurrentQueue<LightStorage> *newOutputQueuePtr)
{
    outQPtrs.push_back(newOutputQueuePtr);
}

LightStorage *LightFinder::findLight(const FrameInfo &frame)
{
    Mat temp;
    LightStorage *result = new LightStorage(frame.capTime,
                                            frame.rotationVec,
                                            frame.translationVec,
                                            temp,
                                            frame.sourceCamPtr);

    //preprocessing start
    //TODO: improvements

    medianBlur(frame.img, result->preprocessedImg, 3);
    static Mat ele = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(result->preprocessedImg, result->preprocessedImg, MORPH_CLOSE, ele);

    if (settings.adSetting.enemyColor == ArmorColor_BLUE)
    {
        inRange(result->preprocessedImg,
                settings.adSetting.BGRMinBlue,
                settings.adSetting.BGRMaxBlue,
                result->preprocessedImg);
    }
    else
    {
        inRange(result->preprocessedImg,
                settings.adSetting.BGRMinRed,
                settings.adSetting.BGRMaxRed,
                result->preprocessedImg);
    }

    //preprocessing end

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(result->preprocessedImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    //for each contours
    for (int i = 0; i < contours.size(); i++)
    {
        //find bounding rect
        RotatedRect tempRRect = minAreaRect(Mat(contours[i]));
        ConvertRRectAngle2Normal(tempRRect);
        if (testAspectRatio(tempRRect) &&
            testArea(tempRRect) &&
            testTilt(tempRRect))
        {
            result->lights.push_back(Light(tempRRect));
        }
    }
    return result;
};

bool LightFinder::testAspectRatio(const RotatedRect &light)
{
    return ((light.size.height / light.size.width) >= settings.adSetting.light_min_aspect_ratio_);
};

bool LightFinder::testArea(const RotatedRect &light)
{
    return (light.size.area() >= settings.adSetting.light_min_area_);
};

bool LightFinder::testTilt(const RotatedRect &light)
{
    return (abs(light.angle) <= settings.adSetting.light_max_tilt_);
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

bool LightFinder::tryWork()
{
    if (lock.try_lock())
    {
        FrameInfo *tempin;
        if (inputQ->dequeue(tempin))
        {
            lss.reset();
            if (outQPtrs.size() > 0)
            {
                LightStorage *tempout = findLight(*tempin);
                outQPtrs[0]->enqueue(tempout);
                for (int i = 1; i < outQPtrs.size(); i++)
                {
                    outQPtrs[i]->enqueue(new LightStorage(*tempout));
                }
            }
            delete tempin;
            lss.lap();

            lock.unlock();
            return true;
        }
        lock.unlock();
    }
    return false;
};

ArmorProcessor::ArmorProcessor(ConcurrentQueue<LightStorage> *_inputQ)
    : inputQ(_inputQ){};

void ArmorProcessor::addOutputQueue(ConcurrentQueue<ArmorStorage> *newOutputQueuePtr)
{
    outQPtrs.push_back(newOutputQueuePtr);
}

bool ArmorProcessor::tryWork()
{
    if (lock.try_lock())
    {
        LightStorage *tempin;
        if (inputQ->dequeue(tempin))
        {
            ads.reset();
            if (outQPtrs.size() > 0)
            {
                ArmorStorage *tempout = generateArmors(*tempin);
                outQPtrs[0]->enqueue(tempout);
                for (int i = 1; i < outQPtrs.size(); i++)
                {
                    outQPtrs[i]->enqueue(new ArmorStorage(*tempout));
                }
            }
            delete tempin;
            ads.lap();

            lock.unlock();
            return true;
        }
        lock.unlock();
    }
    return false;
};

ArmorStorage *ArmorProcessor::generateArmors(const LightStorage &lights)
{
    ArmorStorage *out = new ArmorStorage(lights.capTime, lights.preprocessedImg.size());
    vector<ArmorLightGp> algps;

    armorGrouper(lights, algps);
    armorLocator(algps, lights.sourceCamPtr, *out);
    return out;
};

//grouping functions
void ArmorProcessor::armorGrouper(const LightStorage &ls, vector<ArmorLightGp> &algps)
{
    algps.clear();
    for (vector<Light>::const_iterator j = ls.lights.begin(); j != ls.lights.end(); j++)
    {
        //serach through found armor groups and match with the previously found armor groups
        vector<ArmorLightGp>::iterator firstMatchArmor;
        bool matched = false;

        for (vector<ArmorLightGp>::iterator i = algps.begin(); i != algps.end();)
        {
            if (matchLightWithArmor(*i, *j))
                if (matched)
                {
                    merge(*firstMatchArmor, *i);
                    i = algps.erase(i);
                    continue;
                }
                else
                {
                    matched = true;
                    firstMatchArmor = i;
                    addLightToArmor(*i, j.base());
                }
            i++;
        }

        //no matching armor found, create new armor object
        if (!matched)
        {
            ArmorLightGp newALGP(j.base());
            algps.push_back(newALGP);
        }
    }
};

void ArmorProcessor::armorLocator(const vector<ArmorLightGp> &ArmorLightGps,
                                  const Camera *const &sourceCamPtr,
                                  ArmorStorage &result)
{
    cv::Vec3d rotationVec, translationVec;
    for (vector<ArmorLightGp>::const_iterator i = ArmorLightGps.begin(); i != ArmorLightGps.end(); i++)
    {
        //only handle armor group with 2 lights now
        if (i->lights.size() == 2)
        {
            vector<Point2f> vertices;
            i->getVertices(vertices);
            solvePnP(settings.adSetting.realArmorPoints,
                     vertices,
                     sourceCamPtr->getCameraMatrix(),
                     sourceCamPtr->getDistCoeffs(),
                     rotationVec,
                     translationVec);
            //TODO: apply transformation and rotation for camera
            sourceCamPtr->rectifyCoor(translationVec);

            result.armors.push_back(Armor(rotationVec, translationVec, vertices));

#ifdef DEBUG
            Point2f temp;
            for (int j = 0; j < vertices.size(); j++)
            {
                temp += vertices[j];
            }
            temp.x /= 4;
            temp.y /= 4;
            cout << "Found armor at \n  Pixel: " << temp << "\n  Coordinate:\n  "
                 << translationVec << endl;
#endif
        }
    }
};

void ArmorProcessor::merge(ArmorLightGp &main, ArmorLightGp &tool)
{
    vector<const Light *>::iterator i, j;
    i = main.lights.begin();
    for (j = tool.lights.begin(); j != tool.lights.end(); j++)
    {
        for (; i != main.lights.end(); i++)
            if ((*i)->rect.center.x > (*j)->rect.center.x)
                break;
        i = main.lights.insert(i, *j);
    }
};
bool ArmorProcessor::matchLightWithArmor(const ArmorLightGp &gp, const Light &light) const
{
    vector<const Light *>::const_iterator i;
    for (i = gp.lights.begin(); i != gp.lights.end(); i++)
    {
        //cout << "matching [" << light.vertex[0] << light.vertex[1] << " with " << (*i)->vertex[0] << (*i)->vertex[1] << endl;
        // bool a = testSeparation(*i, &light),
        //      b = testTilt(*i, &light),
        //      c = testSize(*i, &light);
        // //cout << a << b << c << endl;

        if (testSeparation(*i, &light) &&
            testTilt(*i, &light) &&
            testSize(*i, &light))
        {
            return true;
        }
    }
    return false;
};

bool ArmorProcessor::testSeparation(const Light *a, const Light *b) const
{
    double meanLength = 0.5 * (a->rect.size.height + b->rect.size.height);
    //cout << "meanLength " << meanLength << endl;
    double distance = norm(a->rect.center - b->rect.center);
    // cout << "distance " << distance << endl;
    //cout << "settings.adSetting.armor_max_aspect_ratio_" << settings.adSetting.armor_max_aspect_ratio_ << endl;
    return (distance <= (settings.adSetting.armor_max_aspect_ratio_ * meanLength));
};

bool ArmorProcessor::testTilt(const Light *a, const Light *b) const
{
    if (abs(a->rect.angle - b->rect.angle) <= settings.adSetting.armor_max_tilt_diff_)
    {
        double angle = atan2(a->rect.center.y - b->rect.center.y,
                             a->rect.center.x - b->rect.center.x) *
                       180.0 / CV_PI;
        if (angle > 90)
            angle -= 180.0;

        float d1 = abs(a->rect.angle - angle),
              d2 = abs(b->rect.angle - angle);

        return (d1 <= settings.adSetting.armor_max_anglePos_diff_ && d2 <= settings.adSetting.armor_max_anglePos_diff_);
    }
    return false;
}; //check both the orientation of lights, and the angle formed between them

bool ArmorProcessor::testSize(const Light *a, const Light *b) const
{
    float height = a->rect.size.height * b->rect.size.height;
    float distance = norm(a->rect.center - b->rect.center);
    return (height * distance >= settings.adSetting.armor_min_area_);
};
bool ArmorProcessor::addLightToArmor(ArmorLightGp &gp, const Light *newlight)
{
    vector<const Light *>::iterator i;
    for (i = gp.lights.begin(); i != gp.lights.end(); i++)
        if ((*i)->rect.center.x >= newlight->rect.center.x)
        {
            gp.lights.insert(i, newlight);
            return true;
        }
    //the new light is the right-most:
    gp.lights.push_back(newlight);
    return true;
};

ArmorProcessor::ArmorLightGp::ArmorLightGp(const Light *firstlight)
{
    lights.push_back(firstlight);
};
ArmorProcessor::ArmorLightGp::~ArmorLightGp(){};
void ArmorProcessor::ArmorLightGp::coutInfo() const {};
void ArmorProcessor::ArmorLightGp::paintOnMat(Mat &img)
{
    for (int i = 0; i < this->lights.size(); i++)
    {
        if (i > 0)
        {
            line(img,
                 lights[i]->vertex[0],
                 lights[i - 1]->vertex[0],
                 armorDrawColor);
            line(img,
                 lights[i]->vertex[1],
                 lights[i - 1]->vertex[1],
                 armorDrawColor);
        }
        line(img,
             lights[i]->vertex[0],
             lights[i]->vertex[1],
             armorDrawColor);
    }
};

void ArmorProcessor::ArmorLightGp::getVertices(vector<Point2f> &points) const
{
    points.clear();
    for (int i = 0; i < this->lights.size(); i++)
    {
        points.push_back(lights[i]->vertex[0]);
        points.push_back(lights[i]->vertex[1]);
    }
};
