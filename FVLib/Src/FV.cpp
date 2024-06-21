#include <cmath>
using namespace std;

#include "global.h"
#include "FV.h"
#include "Broadcast.h"
#include "PathPoint.h"
#include "GlobalSituation.h"


// Method for broadcasting
// broadcastPlan: if true, the the short plan is broadcast unconditionally
void FV::doBroadcast(bool broadcastPlan) {
  if (check::LE(nextBroadcastInstant, time)) {
    nextBroadcastInstant += broadcastStep;
    FVState state = getState();
    globalSituation->aetherInfo.broadcastState(name, time, getState());

    nextBroadcastInstant += broadcastStep;
  }

  if (broadcastPlan ||
    !globalSituation->aetherInfo.shortPlans.contains(name) ||
    globalSituation->aetherInfo.shortPlans[name].shortPlan.isEmpty() ||
    check::LE(globalSituation->aetherInfo.shortPlans[name].translationTime, time)) {
    FVState curState = getState();
    FlightPlan shortPlan;
    shortPlan.addPoint(PathPoint(curState.position, time));

    for (int l = currentPath.size(), i = 0;
      i < l && shortPlan.size() <= AEtherInfo::shortPlanLength;
      i++) {

      const PathPoint& p = currentPath[i];
      if (check::LE(p.arrivalTime, time)) continue;

      shortPlan.addPoint(p);
    }

    globalSituation->aetherInfo.broadcastPlan(name, time, shortPlan);
  }
}
