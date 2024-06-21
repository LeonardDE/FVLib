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
  
  ~GlobalSituation() {
    for (FV* p : FVs) {
      delete p;
    }
  }
};