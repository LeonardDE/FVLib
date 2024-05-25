#include "Modeler.h"
#include "DataParser.h"
#include "OutputData.h"
Modeler::Modeler(string file_name)
{
	// Собираем данные о ЛА
	ParseFVToList(file_name, globalSituation);
	// Собираем данные для вычислений
	ParseSolveDataFromJSON(file_name, solve_time, integral_h, solve_part_count);

	// Ищем максимальное и минимальное значение времени
	for (auto& fv : globalSituation.FVs)
	{
		for (auto& point : fv->getDynamicPath().getPath())
		{
			start_time = start_time > point.arrivalTime ? point.arrivalTime : start_time;
			end_time = end_time < point.arrivalTime ? point.arrivalTime : end_time;
		}
	}
}

void Modeler::startModeling()
{
	double timer = start_time;
	double h = solve_time / solve_part_count;

	OutputJsonData data;
	for (auto& fv : globalSituation.FVs)
	{
		OutputFV out_fv;
		Plane plane = fv->getPlane();
		out_fv.parameters.type = plane.type;
		out_fv.parameters.name = plane.name;
		out_fv.parameters.plan = vector<Point>();
		data.FVs.push_back(out_fv);

	}

	while (timer < end_time)
	{
		timer += solve_time;
		for (auto& fv : globalSituation.FVs)
		{
			fv->next(h, timer);
			//fv->writeFVState(solve_time);

		}
	
		for (int i = 0; i < data.FVs.size(); i++)
		{
			FV* fv = globalSituation.FVs[i];
			OutputFV& out_fv = data.FVs[i];
			DynamicData dynamic_data;
			Plane plane = fv->getPlane();
			dynamic_data.t = timer;
			dynamic_data.x = plane.x;
			dynamic_data.y = plane.y;
			dynamic_data.z = plane.z;
			dynamic_data.v_x = plane.speedX;
			dynamic_data.v_y = plane.speedY;
			dynamic_data.v_z = plane.speedZ;
			for (auto& p : fv->getDynamicPath().getPath())
			{
				dynamic_data.new_plan.push_back(Point(p.position,p.arrivalTime));
			}

			out_fv.dynamic_data.push_back(dynamic_data);

		}
	}
	writeJsonData("output.json", data);
}