#pragma once
#include <iostream>
#include <chrono>
#include <ctime>
#include <string>

using namespace std;

class StopWatch
{
public:
  StopWatch(string name);
  void lap();                  //cout time taken and smoothed fps
  void check(const int &flag); //cout the time taken from last lap/reset
  void reset();

private:
  string name;

  std::chrono::time_point<std::chrono::steady_clock> lastTime;
  double smoothedDeltaTime;
};

