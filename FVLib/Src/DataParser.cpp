#include "DataParser.h"
#include "GlobalSituation.h"
#include "MaterialPoint.h"
#include "Copter.h"


void ParseSolveDataFromJSON(string json_name, double& time_step, double& integration_h, int& solve_part_count)
{
	ifstream f(json_name);
	json data = json::parse(f);

	time_step = data["timeStep"];
	solve_part_count = data["solve_part_count"];
	f.close();
};

void ParsePathFromJSON(const json& path_data, vector<Point>& path)
{
	double time = -INFINITY;
	for (auto& point_data : path_data)
	{
		if (time > point_data["time"])
		{
			cout << "PathList is not sorting by ASC" << endl;
			path.clear();
			exit(0);
		}
		path.push_back(
			Point(
				Vector3(point_data["x"], point_data["y"], point_data["z"]),
				point_data["time"]
			)
		);
		time = point_data["time"];
		cout << path[path.size() - 1].arrivalTime << endl;
		path[path.size() - 1].position.print();
	}
};

void ParseFVToList(string json_name, GlobalSituation& gs)
{
	ifstream f(json_name);
	json data = json::parse(f);

	for (auto& sub_j : data["AirCraftList"])
	{
		cout << "---------" << endl;
		cout << sub_j << endl;
		cout << "---------" << endl;

		auto& fv_param = sub_j["parameters"];

		// Чтение пути для БПЛА
		auto& fv_path = sub_j["path"];
		vector<Point> path;
		ParsePathFromJSON(fv_path, path);
		//

		if (sub_j["type"] == "MaterialPoint")
		{
			MaterialPoint* point = new MaterialPoint(
				sub_j["fv_id"],
				path[0].position.x, path[0].position.y, path[0].position.z,
				fv_param["v_x"], fv_param["v_y"], fv_param["v_z"],
				fv_param["maxAcceleration"], fv_param["k_xz"], fv_param["k_y"],
				fv_param["broadcastStep"], fv_param["radiusFilter"],
				fv_param["heightWarn"], fv_param["radiusWarn"], &gs);
			point->setPath(path);
			gs.FVs.push_back(point);
		}
		else if (sub_j["type"] == "Copter")
		{
			Copter* copter = new Copter(
				sub_j["fv_id"],
				path[0].position.x, path[0].position.y, path[0].position.z,
				fv_param["v_x"], fv_param["v_y"], fv_param["v_z"],
				fv_param["inertial_xz"], fv_param["inertial_y"],
				fv_param["k_xz"], fv_param["k_y"],
				fv_param["maxVelocity_xz"], fv_param["maxVelocity_y"], fv_param["minVelocity_y"],
				fv_param["broadcastStep"], fv_param["radiusFilter"],
				fv_param["heightWarn"], fv_param["radiusWarn"], &gs);

			copter->setPath(path);
			gs.FVs.push_back(copter);
		}
	}
	f.close();
};