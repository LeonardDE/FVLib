#pragma once
#include "Vector3.h"

// The structure of a control point: instant, coordinates
struct Point {
  Point(Vector3 position = Vector3(0, 0, 0), double arrivalTime = 0) : position(position), arrivalTime(arrivalTime) {};

  // The passing instant
  double arrivalTime;

  // The location of the point
  Vector3 position;
};


// The type of a point in the navigator plan
enum PointType {
  // A point of straight part beginning 
  DEFAULT,

  // The turn start point in the circular conjuction
  START_TURN
};

