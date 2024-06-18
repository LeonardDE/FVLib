#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <fstream>

#include "Vector3.h"
#include "Path.h"
#include "Plane.h"

#include "lib/json.h"
using json = nlohmann::json;

// Forward declaration of the global situation class
// to declare a reference to the object from an FV instance
class GlobalSituation;

// The basic abstract class of a FV
class FV {
public:

  // Method to take data of the current position for final writing
  virtual FVOutputState getOutputState() = 0;

  // Method to integrate motion of the FV up to the given time with the given time step
  virtual void next(double h, double end_time) = 0;

  virtual void setPath(vector<Point> path);
  
  vector<Point> getBasePath() {
    return basePath;
  };
  Path getDynamicPath() {
    return turnPath;
  };
  double getTime() {
    return time;
  }
  FVType getType() {
    return type;
  }
  string getName() {
    return name;
  }

  friend void mergeShortPlans(const vector<Point>& arr1, const vector<Point>& arr2,
    vector<Point>& res1, vector<Point>& res2);
  json conflicts;

protected:
  // The pointer to the global data structure
  GlobalSituation* globalSituation;

  // Personal name of the FV
  string name;

  // The type of the FV
  FVType type;

  // The apriori flight plan
  vector<Point> basePath;

  // The actual flight plan
  vector<Point> dynamicPath;

  // The navigator plan
  Path turnPath;

  // Current time of the vehicle
  double time = 0;

  // Time step for broadcasting the current state
  double broadcastStep;

  // Instant of the next broadcast
  double nextBroadcastInstant = 1e6;

  // Method for broadcasting
  virtual void doBroadcast();

  double radiusFilter;
  double radiusWarn;
  double heightWarn;

  virtual double solveTurnRadius(const Vector3& v1, const Vector3& v2) = 0;



  virtual void checkConflict();
};





void mergeShortPlans(const vector<Point>& arr1, const vector<Point>& arr2,
  vector<Point>& res1, vector<Point>& res2);