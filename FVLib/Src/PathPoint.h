#pragma once
#include "Point.h"
#include "TurnData.h"
struct PathPoint : public Point
{
    PathPoint(
        Vector3 position = Vector3(0, 0, 0),
        double arrivalTime = 0,
        PointType type = PointType::DEFAULT, TurnData turnData = TurnData()) :
        position(position), arrivalTime(arrivalTime), type(type), turnData(turnData) {};

    PathPoint(
        Point point = Point(),
        PointType type = PointType::DEFAULT,
        TurnData turnData = TurnData()) : position(point.position), arrivalTime(point.arrivalTime), type(type), turnData(turnData) {};

    Vector3 position;
    double arrivalTime;

    PointType type;
    TurnData turnData;

};
