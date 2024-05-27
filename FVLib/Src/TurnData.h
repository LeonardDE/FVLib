#pragma once
#include "Vector3.h"
struct TurnData
{
    TurnData(
        double w = 0, double eps = 0, double w_0 = 0,
        Vector3 axis = Vector3().Zero(),
        Vector3 axisPoint = Vector3().Zero()) :
        angularVelocity(w), angularAcceleration(eps), angularVelocity_0(w_0), axis(axis), axisPoint(axisPoint) {};

    double angularVelocity;
    double angularAcceleration;
    Vector3 axis;
    Vector3 axisPoint;

    double angularVelocity_0;
};