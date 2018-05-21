/**
 * @brief 
 * 
 * @file ArmorTracker.hpp
 * @author Alex Au
 * @date 2018-03-28
 */
#pragma once
#include "defines.hpp"
#include "ArmorDetection.hpp"
#include "LinearRegression.hpp"

/**
  * @brief 
  * a class used to store detection history of a specific armor
 */
class TrackedArmor
{
public:
  TrackedArmor(const timeval &obvTime);
  TrackedArmor(const TrackedArmor &copy_source);
  TrackedArmor &operator=(const TrackedArmor &copy_src);
  bool match_add(const Armor &armor, const timeval &obvTime);
  /**
   * @brief 
   * push back the new armor and update the prediction formula
   * @param armor 
   * @param obvTime 
   */
  void push_back(const Armor &armor, const timeval &obvTime);

  /**
  * @brief 
  * predict the armor coordinate at time t
  * @param t 
  * @param predictXYZ the predicted coordinate 
  * @return true 
  * @return false probably something is broken
  */
  bool predict(const timeval &t, cv::Vec3d &predictXYZ);

  double getDistance(const timeval &t);

  timeval getLastSeen();

  void print();

private:
  LinearRegression<3> prediction;
  timeval lastseen;
  timeval spotTime;
};

class ArmorTracker //perform logging and tracking
{
public:
  ArmorTracker();

  void addSource(ConcurrentQueue<ArmorStorage> *const source);

  /**
  * @brief Get the Valid Target object
  *   
  * @param targets : a vector of copies of tracked armor that should be real
  * @return true 
  * @return false 
 */
  void getValidatedTargets(vector<TrackedArmor> &targets);

  bool tryWork();

  vector<TrackedArmor *> trackedArmors;

private:
  /**
   * @brief 
   * matching and updating the observed armors with the armor history
   * @param newArmors 
   */
  void update(ArmorStorage &newArmors);

  vector<ConcurrentQueue<ArmorStorage> *> sources;
  mutex lock;
};
