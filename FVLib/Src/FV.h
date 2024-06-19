#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <fstream>

using namespace std;

#include "Vector3.h"
#include "Plane.h"
#include "NavigationPlan.h"
#include "RectilinearSegment.h"
#include "CircularSegment.h"
#include "PathPoint.h"
#include "FlightPlan.h"

#include "lib/json.h"

using json = nlohmann::json;


// Forward declaration of the global situation class
// to declare a reference to the object from an FV instance
class GlobalSituation;


// The basic abstract class of a FV
class FV {
protected:
  // The pointer to the global data structure
  GlobalSituation* globalSituation;

  // Personal name of the FV
  string name;

  // The type of the FV
  FVType type;

  // Method for navigation near checkpoints
  NavigationMethods navType;

  // The apriori flight plan
  FlightPlan basePath;

  // The actual flight plan
  FlightPlan currentPath;

  // The navigator plan
  NavigationPlan navigationPath;

  // Current time of the vehicle
  double time = 0;

  // Time step for broadcasting the current state
  double broadcastStep;

  // Instant of the next broadcast
  double nextBroadcastInstant = 1e6;

  // Method for broadcasting
  virtual void doBroadcast();

  // Distance to filter FVs for further procession
  double radiusFilter;

  // The parameters of the safety zone of the FV - the radius of the cylinder
  double safetyRadius;

  // The parameters of the safety zone of the FV - the semi-height of the cylinder
  double safetyHeight;

public:
  // Method to take data of the current position for final writing
  virtual FVOutputState getOutputState() = 0;

  // Method to integrate motion of the FV up to the given time with the given time step
  virtual void next(double h, double end_time) = 0;

  // Getting the base flight plan of the FV
  virtual const FlightPlan& getBasePath() const {
    return basePath;
  };

  // Getting the current flight plan of the FV
  virtual const FlightPlan& getCurrentPath() const {
    return currentPath;
  };

  // Getting the navigation path
  const NavigationPlan& getNavigationPath() {
    return navigationPath;
  };

  // Getting the current time of the FV
  virtual double getTime() const {
    return time;
  }

  // Getting name of the FV
  virtual const string& getName() const {
    return name;
  }

  // Getting the dynamics type of the FV
  virtual FVType getType() const {
    return type;
  }

  // Getting the navigation type of the FV
  virtual NavigationMethods getNavigationType() const {
    return navType;
  }

};
