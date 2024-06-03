#include "GlobalSituation.h"

void AEtherInfo::broadcastState(const Plane& plane)
{
	states[plane.name].plane = plane;
}

void AEtherInfo::broadcastPlan(const string& name,double time, const vector<Point>& shortPlan)
{
	states[name].shortPlan = shortPlan;
	states[name].translationTime = time;
	cout << "TRANSLATION TIME "  << time << endl;
	
}