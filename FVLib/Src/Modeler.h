#pragma once
#include "FV.h"
#include "GlobalSituation.h"
#include <limits>

class Modeler
{
public:
	Modeler(string file_name);

	void startModeling();
	
private:
	// Перенести в GlobalSituation
	//vector<FV*> fv_list;

	double solve_time;
	double integral_h;
	int solve_part_count;
	double start_time = std::numeric_limits<double>::max();
	double end_time = std::numeric_limits<double>::min();

	GlobalSituation globalSituation;
};