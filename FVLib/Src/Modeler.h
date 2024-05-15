#pragma once
#include "FV.h"
#include <limits>
class Modeler
{
public:
	Modeler(string file_name);
	~Modeler()
	{
		for (auto p : fv_list)
		{
			delete p;
		}
	};

	void startModeling();
	
private:
	vector<FV*> fv_list;

	double solve_time;
	double integral_h;
	int solve_part_count;
	double start_time = std::numeric_limits<double>::max();;
	double end_time = std::numeric_limits<double>::min();;
};