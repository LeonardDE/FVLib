#include "GlobalSituation.h"

void AEtherInfo::broadcastState(double time, const FVState& plane) {
  states[plane.name].plane = plane;
  states[plane.name].planeTranslationTime = time;
}

void AEtherInfo::broadcastPlan(const string& name, double time, const vector<Point>& shortPlan) {
  states[name].shortPlan = shortPlan;
  states[name].translationTime = time;
}