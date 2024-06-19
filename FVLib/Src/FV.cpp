#include <cmath>
using namespace std;

#include "global.h"
#include "FV.h"
#include "GlobalSituation.h"


void FV::doBroadcast() {
  if (check::LE(nextBroadcastInstant, time)) {
    nextBroadcastInstant += broadcastStep;
    FVOutputState plane = getOutputState();
    globalSituation->aetherInfo.broadcastState(time, plane);

    nextBroadcastInstant += broadcastStep;
  }


  if (globalSituation->aetherInfo.states[this->name].shortPlan.empty() ||
    check::LE(globalSituation->aetherInfo.states[this->name].shortPlan[0].arrivalTime, time)) {
    FVOutputState plane = getOutputState();
    vector<Point> short_plan;
    for (auto& p : dynamicPath) {

      if (short_plan.size() > 3) break;
      if (check::LE(p.arrivalTime, time)) continue;
      short_plan.push_back(Point(p.position, p.arrivalTime));
    }

    globalSituation->aetherInfo.broadcastPlan(plane.name, time, short_plan);
  }
}
