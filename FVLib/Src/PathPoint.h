#pragma once
#include "Vector3.h"

// The structure of a control point: instant, coordinates
struct PathPoint {
  PathPoint(Vector3 position = Vector3(0, 0, 0), double arrivalTime = 0) : position(position), arrivalTime(arrivalTime) {};

  // The passing instant
  double arrivalTime;

  // The location of the point
  Vector3 position;
};


