#include <stdexcept>
using namespace std;

#include "DataParser.h"

#include "GlobalSituation.h"

#include "Plane.h"
#include "FlightPlan.h"

#include "MassPoint.h"
#include "Copter.h"


void ParseSolveDataFromJSON(const string& json_name, double& time_step, double& integration_h, int& solve_part_count) {
  ifstream f(json_name);
  json data = json::parse(f);

  time_step = data["timeStep"];
  solve_part_count = data["solve_part_count"];
  f.close();
};

void ParsePathFromJSON(const json& path_data, FlightPlan& path) {
  double time = -INFINITY;
  for (auto& point_data : path_data) {
    if (time > point_data["time"]) {
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
  }
};

void ParseFVToList(const string& json_name, GlobalSituation& gs) {
  ifstream f(json_name);
  json data = json::parse(f);

  for (auto& sub_j : data["AirCraftList"]) {

    auto& fv_param = sub_j["parameters"];

    auto& fv_path = sub_j["path"];
    vector<Point> path;
    ParsePathFromJSON(fv_path, path);

    string typeName = sub_j["type"];
    if (!FVNameToType.contains(sub_j["type"])) {
      throw invalid_argument("Unknown vehicle type '" + typeName + "'");
    }
    FVType type = FVNameToType[typeName];

    switch (type) {
    case MASSPOINT: {
      MassPoint* point = new MassPoint(
        sub_j["fv_id"],
        path[0].position.x, path[0].position.y, path[0].position.z,
        fv_param["v_x"], fv_param["v_y"], fv_param["v_z"],
        fv_param["maxAcceleration"], fv_param["k_xz"], fv_param["k_y"],
        fv_param["broadcastStep"], fv_param["radiusFilter"],
        fv_param["heightWarn"], fv_param["radiusWarn"], &gs);
      point->setPath(path);
      gs.FVs.push_back(point);
      break;
    }

    case COPTER: {
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
      break;
    }

    default:
      throw invalid_argument("The procession of the type '" + typeName + "' is not implemented");
    }
  }
  f.close();
};