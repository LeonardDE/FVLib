#pragma once
#include <vector>
#include <string>
#include <map>

#include "FV.h"
#include "Vector3.h"

struct FVState
{
	Plane plane;

	//bool isExist = false;
	vector<Point> shortPlan;
	
};

class AEtherInfo
{
public:
	map<string,FVState> states;
	void broadcastState(const Plane& plane);
	void broadcastPlan(const string& name, const vector<Point>& shortPlan);
};

class GlobalSituation
{
public:
	vector<FV*> FVs;
	AEtherInfo aetherInfo;
	~GlobalSituation()
	{
		for (auto p : FVs)
		{
			delete p;
		}
	};
private:
};