#include <string>
#include <stdexcept>
using namespace std;

#include "GlobalSituation.h"

// Adding a FV into the situation
void GlobalSituation::addFV(FV* fv) {
  string name = fv->getName();
  if (nameToIndex.contains(name))
  {
    throw invalid_argument("GlobalSituation::addFV: trying to add a FV with a name equal to the one of a previously added FV!");
  }

  nameToIndex[name] = FVs.size();
  FVs.push_back(fv);
}

// Method to broadcast the state of a FV
void GlobalSituation::broadcastState(const int ind, double time, const FVState& state) {
  string name = FVs[ind]->getName();

  // Write data to the current state of the aether
  aetherInfo.states[name].state = state;
  aetherInfo.states[name].translationTime = time;

  // Write data to the output flow of broadcasts
  PositionBroadcastData brState;
  brState.t = time;
  brState.x = state.position.x;
  brState.y = state.position.y;
  brState.z = state.position.z;
  brState.vx = state.velocity.x;
  brState.vy = state.velocity.y;
  brState.vz = state.velocity.z;
  jsonData.FVs[ind].broadcasts.positions.push_back(brState);
}

// Method to broadcast the shortPlan of a FV
void GlobalSituation::broadcastPlan(const int ind, double time, const FlightPlan& shortPlan) {
  string name = FVs[ind]->getName();

  // Write data to the current state of the aether
  aetherInfo.shortPlans[name].shortPlan = shortPlan;
  aetherInfo.shortPlans[name].translationTime = time;

   // Write data to the output flow of broadcasts
   PlanBroadcastData planData;
  planData.t = time;
  for (int l = shortPlan.size(), i = 0; i < l; i++) {
    planData.plan.push_back(shortPlan[i]);
  }
  jsonData.FVs[ind].broadcasts.plans.push_back(planData);
}