#pragma once

#include <string>
#include <limits>

#include "GlobalSituation.h"
#include "OutputData.h"

class Modeler
{
public:
	Modeler(const string& in_file, const string& out_file);

	void startModeling();
	
private:
	double timeStep;
	double integral_h;
	int solve_part_count;
	double start_time = std::numeric_limits<double>::max();
  double end_time = std::numeric_limits<double>::min();

  string in_file;
  string out_file;

  GlobalSituation globalSituation;
  OutputJsonData data;
};