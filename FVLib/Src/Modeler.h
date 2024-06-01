#pragma once
#include <string>
#include <limits>
#include "GlobalSituation.h"


class Modeler
{
public:
	Modeler(string file_name);

	void startModeling();
	
private:
	double timeStep;
	double integral_h;
	int solve_part_count;
	double start_time = std::numeric_limits<double>::max();
	double end_time = std::numeric_limits<double>::min();

	GlobalSituation globalSituation;
};