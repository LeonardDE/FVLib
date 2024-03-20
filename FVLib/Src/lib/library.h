#include "flight_plan.h"
#include "plane.h"

#ifndef DRONE_LIB_LIBRARY_H
#define DRONE_LIB_LIBRARY_H

Scheme getScheme();

void subscribeAirSituation(StateObserver stateObserver);

void startSimulation();

void pauseSimulation();

void stopSimulation();

void setTime(int32_t time);

#endif //DRONE_LIB_LIBRARY_H
