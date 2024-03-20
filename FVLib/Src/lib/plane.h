#include <iostream>
#include "vector"

#ifndef DRONE_LIB_PLANE_H
#define DRONE_LIB_PLANE_H

using namespace std;

struct Plane
{
    string name;
    double x;
    double z;
    double y;
    double speedX;
    double speedZ;
    double speedY;
    double speed;
    int32_t type;
};

struct AirSituationState
{
    int32_t time;
    vector<Plane> planes;
};

typedef void (*StateObserver)(AirSituationState);

#endif //DRONE_LIB_PLANE_H
