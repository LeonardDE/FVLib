#pragma once
#include "Vector3.h"
enum class PointType
{
    DEFAULT = 0,
    START_TURN = 1
};

struct Point
{
    Point(Vector3 position = Vector3(0, 0, 0), double arrivalTime = 0) : position(position), arrivalTime(arrivalTime) {};

    Vector3 position;
    double arrivalTime;
};