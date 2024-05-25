#include "GlobalSituation.h"

void AEtherInfo::broadcastState(const Plane& plane)
{
	states[plane.name].plane = plane;
}

void AEtherInfo::broadcastPlan(const string& name, const vector<Point>& shortPlan)
{
	states[name].shortPlan = shortPlan;
	//states[name].shortPlan = vector<Point>();
}