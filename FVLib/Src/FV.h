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
  virtual FVState getState() = 0;

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
  // Personal name of the FV
  string name;

  // The type of the FV
  FVType type;

  
  vector<Point> basePath;
  vector<Point> dynamicPath;
  Path turnPath;
  double time = 0;

  double radiusFilter = 0;
  double broadcastStep;
  double nextBroadcastInstant = 0;

  double radiusWarn;
  double heightWarn;

  virtual double solveTurnRadius(const Vector3& v1, const Vector3& v2) = 0;

  virtual void doBroadcast();

  GlobalSituation* globalSituation;

  virtual void checkConflict();
};





void mergeShortPlans(const vector<Point>& arr1, const vector<Point>& arr2,
  vector<Point>& res1, vector<Point>& res2);