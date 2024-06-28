#pragma once
#include <vector>
#include <string>
#include <map>
using namespace std;

#include "Plane.h"
#include "PathPoint.h"
#include "FV.h"
#include "FlightPlan.h"
#include "Broadcast.h"


class GlobalSituation {
public:
  vector<FV*> FVs;
  AEtherInfo aetherInfo;
  OutputJsonData jsonData;
  map<string, int> nameToIndex;

  // Adding a FV into the situation
  void addFV(FV* fv);

  // The destructor
  ~GlobalSituation() {
    for (FV* p : FVs) {
      delete p;
    }
  }

  // Method to broadcast the state of a FV
  void broadcastState(const int ind, double time, const FVState& state);

  // Method to broadcast the shortPlan of a FV
  void broadcastPlan(const int ind, double time, const FlightPlan& shortPlan);
};