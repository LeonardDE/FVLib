#include "Broadcast.h"

// Number of future checkpoint to be broadcast in a short plan
const int AEtherInfo::shortPlanLength = 4;


// Method to broadcast the state of a FV
void AEtherInfo::broadcastState(const string& name, double time, const FVState& state) {
  states[name].state = state;
  states[name].translationTime = time;
}


// Method to broadcast the shortPlan of a FV
void AEtherInfo::broadcastPlan(const string& name, double time, const FlightPlan& shortPlan) {
  shortPlans[name].shortPlan = shortPlan;
  shortPlans[name].translationTime = time;
}