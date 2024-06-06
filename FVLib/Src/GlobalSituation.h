#pragma once
#include <vector>
#include <string>
#include <map>
#include "lib/plane.h"
#include "Point.h"
#include "FV.h"
using namespace std;


struct FVState
{
	double planeTranslationTime;
	Plane plane;

	//bool isExist = false;
	double translationTime;
	vector<Point> shortPlan;
	
};

class AEtherInfo
{
public:
	map<string,FVState> states;
	void broadcastState(double time,const Plane& plane);
	void broadcastPlan(const string& name,double time, const vector<Point>& shortPlan);
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