#include "Modeler.h"
#include "DataParser.h"
#include "OutputData.h"
#include "FV.h"
#include "global.h"

Modeler::Modeler(string file_name)
{
	// �������� ������ � ��
	ParseFVToList(file_name, globalSituation);
	// �������� ������ ��� ����������
	ParseSolveDataFromJSON(file_name, timeStep, integral_h, solve_part_count);

	// ���� ������������ � ����������� �������� �������
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
	double h = timeStep / solve_part_count;

	OutputJsonData data;
	for (auto& fv : globalSituation.FVs)
	{
		OutputFV out_fv;
		Plane plane = fv->getPlane();
		out_fv.parameters.type = plane.type;
		out_fv.parameters.name = plane.name;
		out_fv.parameters.plan = fv->getBasePath();
		data.FVs.push_back(out_fv);
	}

	while (timer < end_time)
	{
		
		for (auto& fv : globalSituation.FVs)
		{
			fv->next(h, timer);
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

			if (timer + timeStep >= globalSituation.aetherInfo.states[plane.name].translationTime - EPS &&
				timer < globalSituation.aetherInfo.states[plane.name].translationTime - EPS)
			{
				dynamic_data.new_plan = globalSituation.aetherInfo.states[plane.name].shortPlan;
			}
			
			

			out_fv.dynamic_data.push_back(dynamic_data);

		}
		timer += timeStep;
	}
	writeJsonData("output.json", data);
}