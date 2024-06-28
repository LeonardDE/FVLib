#pragma once

#include <map>
#include <string>
using namespace std;

#include "Plane.h"
#include "FlightPlan.h"

// A record type for keeping a state broadcast
struct StateBroadcast {
  double translationTime;
  FVState state;
};


// A record type for keeping a short plan broadcast
struct PlanBroadcast {
  double translationTime;
  FlightPlan shortPlan;
};


// A record type for keeping current broadcast situation
struct AEtherInfo {
  // Number of future checkpoint to be broadcast in a short plan
  static const int shortPlanLength;

  // Storage for last broadcast states of FVs
  map<string, StateBroadcast> states;

  // Storage for last broadcast short plans of FVs
  map<string, PlanBroadcast> shortPlans;
};