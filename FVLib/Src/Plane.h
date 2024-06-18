#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <string>

using namespace std;

// Enumeration of FV types
enum FVType {
  // Material point
  MASSPOINT,

  // Copter
  COPTER,

  // Helicopter with acceleration control
  HELICOPTER_ACC,

  // Helicopter with velocity command signal control
  HELICOPTER_SIGNAL,

  // Aircraft with acceleration control
  AIRCRAFT_ACC,

  // Aircraft with velocity command signal control
  AIRCRAFT_SIGNAL
};


// Structure to convert a string to the FVType enum
extern map<string, FVType> FVNameToType;


// Structure to convert a FVType enum to the string
extern map<FVType, string> FVTypeToName;


// Record of the current FV state for output writing
struct FVState {
  // The ID of the FV
  string name;

  // The current X-coordinate
  double x;

  // The current Z-coordinate
  double z;

  // The current Y-coordinate
  double y;

  // The current X-velocity
  double speedX;

  // The current Z-velocity
  double speedZ;

  // The current Y-velocity
  double speedY;

  // The magnitude of the current velocity
  double speed;

  // The current X-velocity
  string type;
};



