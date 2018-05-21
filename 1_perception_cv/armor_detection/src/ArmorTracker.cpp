/**
 * @brief 
 * 
 * @file ArmorTracker.cpp
 * @author Alex Au
 * @date 2018-03-26
 */
#include "defines.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include "ArmorTracker.hpp"
#include "ArmorDetection.hpp"
#include "StopWatch.hpp"
#include "ros/ros.h"
using namespace std;

StopWatch tsw("trackerUpdateSpeed");
StopWatch tsw2("trackerUpdateFreq");

TrackedArmor::TrackedArmor(const timeval &obvTime)
    : prediction(PREDICT_DEGREE),
      spotTime(obvTime),
      lastseen(obvTime){};

TrackedArmor::TrackedArmor(const TrackedArmor &copy_source)
    : prediction(copy_source.prediction)
{
    this->lastseen = copy_source.lastseen;
    this->spotTime = copy_source.spotTime;
}

TrackedArmor &TrackedArmor::operator=(const TrackedArmor &copy_src)
{
    this->prediction = LinearRegression<3>(copy_src.prediction);
    this->lastseen = copy_src.lastseen;
    this->spotTime = copy_src.spotTime;
    return *this;
};

bool TrackedArmor::match_add(const Armor &armor, const timeval &obvTime)
{
    cv::Vec3d predictCoor;
    predict(obvTime, predictCoor);
#ifdef DEBUG
    cout << "|a-p|^2 / (a.p)"
         << ((pow(armor.translation[0] - predictCoor[0], 2) +
              pow(armor.translation[1] - predictCoor[1], 2) +
              pow(armor.translation[2] - predictCoor[2], 2)) /
             (armor.translation[0] * predictCoor[0] +
              armor.translation[1] * predictCoor[1] +
              armor.translation[2] * predictCoor[2]))
         << endl;
#endif
    if (((pow(armor.translation[0] - predictCoor[0], 2) +
          pow(armor.translation[1] - predictCoor[1], 2) +
          pow(armor.translation[2] - predictCoor[2], 2)) /
         (armor.translation[0] * predictCoor[0] +
          armor.translation[1] * predictCoor[1] +
          armor.translation[2] * predictCoor[2])) <=
        GROUP_SEP_DIST_SQUARE_RATIO)
    {
        push_back(armor, obvTime);
        return true;
    }
    else
        return false;
}

void TrackedArmor::push_back(const Armor &armor, const timeval &obvTime)
{
    double temp = (double)(obvTime.tv_sec - spotTime.tv_sec) + (obvTime.tv_usec - spotTime.tv_sec) / 1000000.0;
#ifdef DEBUG
    cout << "Armor push_back at t = " << temp << endl;
    cout << " with coordinate: " << armor.translation << endl;
#endif
    prediction.decay(PREDICT_DECAY);
    prediction.addValuePair(temp, armor.translation);
    lastseen = obvTime;
}

double TrackedArmor::getDistance(const timeval &t)
{
    cv::Vec3d coor;
    predict(t, coor);
    return sqrt(pow(coor[0], 2) + pow(coor[1], 2) + pow(coor[2], 2));
}

timeval TrackedArmor::getLastSeen()
{
    return lastseen;
}

bool TrackedArmor::predict(const timeval &t, cv::Vec3d &predictXYZ)
{
    double temp = (double)(t.tv_sec - spotTime.tv_sec) + (t.tv_usec - spotTime.tv_sec) / 1000000.0;
    if (prediction.find_y(temp, predictXYZ))
        return true;
    else
        return false;
};

void TrackedArmor::print()
{
#ifdef DEBUG
    cout << "TrackedArmor: Prediction result\n"
         << prediction.getResult() << endl
         << endl;
#endif
};

ArmorTracker::ArmorTracker(){};

void ArmorTracker::addSource(ConcurrentQueue<ArmorStorage> *const source)
{
    lock_guard<mutex> lg(lock);
    sources.push_back(source);
};

bool ArmorTracker::tryWork()
{
    if (lock.try_lock())
    {
        ArmorStorage *tempin;
        for (auto i : sources)
        {
            if (i->dequeue(tempin))
            {
                update(*tempin);
            }
        }
        lock.unlock();
        return true;
    }
    else
        return false;
};

void ArmorTracker::update(ArmorStorage &newArmors)
{
#ifdef DEBUG
    tsw.reset();
#endif
    //find corresponding tracked armor to update, or create a new one
    for (auto i : newArmors.armors)
    {
        bool matched = false;
        auto j = trackedArmors.begin();

        //match the armor i with the first matching tracked armor
        while (j != trackedArmors.end())
        {
            if ((*j)->match_add(i, newArmors.capTime))
            {
#ifdef DEBUG
                ROS_INFO("Updated old armor data, Coordinate at (%f, %f, %f)",
                         i.translation[0],
                         i.translation[1],
                         i.translation[2]);
#endif
                matched = true;
                break;
            }
            j++;
        }

        //create new if none is matched
        if (!matched)
        {
#ifdef DEBUG
            ROS_INFO("New armor found! Coordinate at (%f, %f, %f)",
                     i.translation[0],
                     i.translation[1],
                     i.translation[2]);
#endif
            trackedArmors.push_back(new TrackedArmor(newArmors.capTime));
            trackedArmors.back()->push_back(i, newArmors.capTime);
        }
    }

    //erase trackedArmors that are too old
    auto i = trackedArmors.begin();
    while (i != trackedArmors.end())
    {
        double dt = newArmors.capTime.tv_sec - (*i)->getLastSeen().tv_sec +
                    (newArmors.capTime.tv_usec - (*i)->getLastSeen().tv_usec) / 1000000.0;

        if (dt >= ARMOR_EXPIRE_TIME_SEC)
        {
#ifdef DEBUG
            ROS_INFO("a armor record expired");
#endif
            i = trackedArmors.erase(i);
        }
        else
            i++;
    }

#ifdef DEBUG
    tsw.lap();
#endif
    tsw2.lap();
};

//this better be copying the tracked armors for further modification, so this is blocking
void ArmorTracker::getValidatedTargets(vector<TrackedArmor> &targets)
{
    //TODO: real validation for intelligence
    //now its only returning the closest armor
    lock_guard<mutex> guard(lock);
    targets.clear();
    if (this->trackedArmors.size() > 0)
    {
        double min_d = 0;
        int min_id = 0;
        for (int id = 0; id < trackedArmors.size(); id++)
        {
            double temp_d;
            if ((temp_d = trackedArmors[id]->getDistance(trackedArmors[id]->getLastSeen())) < min_d)
            {
                min_d = temp_d;
                min_id = id;
            }
        }
        targets.push_back(*trackedArmors[min_id]);
    }
}
