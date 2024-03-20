#include <iostream>
#include "vector"

#ifndef DRONE_LIB_FLIGHT_PLAN_H
#define DRONE_LIB_FLIGHT_PLAN_H

using namespace std;

struct CheckPoint
{
    string name;
    int32_t x;
    int32_t z;
    int32_t y;
};

struct FlightPlanSegment
{
    vector<CheckPoint> points;
    FlightPlanSegment* nextSegment;
};

struct FlightPlan
{
    vector<FlightPlanSegment> segments;
    int32_t colorArgb;
};

struct RectangleFlightZone {
    vector<int32_t> x;
    vector<int32_t> z;
    int32_t minY;
    int32_t maxY;
    int32_t colorArgb;
};

struct RoundFlightZone {
    int32_t radius;
    int32_t x;  // x coordinate of the circle's center.
    int32_t z; // z coordinate of the circle's center.
    int32_t minY;
    int32_t maxY;
    int32_t colorArgb;
};

struct Scheme
{
    int32_t minX;
    int32_t maxX;
    int32_t minZ;
    int32_t maxZ;
    RectangleFlightZone flightZone;
    vector<FlightPlan> flightPlans;
    vector<RoundFlightZone> roundFrbiddenZones;
    vector<RectangleFlightZone> rectForbiddenZones;
};

#endif //DRONE_LIB_FLIGHT_PLAN_H
