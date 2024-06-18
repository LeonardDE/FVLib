#pragma once
#include <vector>
#include <string>
#include <map>
using namespace std;

#include "Plane.h"
#include "Point.h"
#include "FV.h"


struct BroadcastBatch {
  double planeTranslationTime;
  FVState plane;
  
  double translationTime;
  vector<Point> shortPlan;
};

class AEtherInfo {
public:
  map<string, BroadcastBatch> states;
  void broadcastState(double time, const FVState& plane);
  void broadcastPlan(const string& name, double time, const vector<Point>& shortPlan);
};

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