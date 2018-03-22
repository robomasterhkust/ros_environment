#include "StopWatch.hpp"
#include <iostream>
#include <chrono>
#include <ctime>
#include <string>
#include <math.h>

using namespace std;

StopWatch::StopWatch(string _name)
    : name(_name),
      lastTime(chrono::steady_clock::now()),
      smoothedDeltaTime(0){};

void StopWatch::check(const int &flag)
{
    std::chrono::time_point<std::chrono::steady_clock> currentTime = chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = currentTime - lastTime;

    cout << name << " reached flag" << flag << " at " << floor(elapsed_seconds.count() * 1000) << "ms\n";
};

void StopWatch::lap()
{
    std::chrono::time_point<std::chrono::steady_clock> currentTime = chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = currentTime - lastTime;
    smoothedDeltaTime = smoothedDeltaTime * 0.8 + elapsed_seconds.count() * 0.2;

    cout << name << " took " << floor(elapsed_seconds.count() * 1000) << "ms = " << floor(1 / smoothedDeltaTime) << "fps\n";
    lastTime = currentTime;
};

void StopWatch::reset()
{
    lastTime = chrono::steady_clock::now();
}