#include <stdexcept>
using namespace std;

#include "global.h"

#include "DataParser.h"

#include "GlobalSituation.h"

#include "Plane.h"
#include "FlightPlan.h"

#include "MassPoint.h"
#include "Copter.h"

const string DataParser::version = "2.0";

DataParser::DataParser(const string &json_name)
{
  ifstream f(json_name);

  if (!f.is_open())
  {
    throw invalid_argument("DataParser constructor: the input file '" + json_name + "' cannot be opened!");
  }

  data = json::parse(f);
  f.close();

  if (!data.contains("version"))
  {
    throw invalid_argument("DataParser constructor: the given input JSON file does not contain version parameter!");
  }

  string givenVersion = data["version"];
  if (givenVersion != version)
  {
    throw invalid_argument("DataParser constructor: the given input JSON file version '" + givenVersion + "' does not equal the necessary version '" + version + "' !");
  }
}

void DataParser::ParseSolveDataFromJSON(double &time_step, int &solve_part_count)
{
  time_step = data["timeStep"];
  solve_part_count = data["solve_part_count"];
};

void DataParser::ParsePathFromJSON(const json &path_data, FlightPlan &path)
{
  path.clear();

  double time = -INFINITY;
  for (auto &point_data : path_data)
  {
    if (check::LE(point_data["time"], time))
    {
      throw invalid_argument("DataParser: a flight plan is not sorted acceding!");
    }
    path.addPoint(
        PathPoint(
            Vector3(point_data["x"], point_data["y"], point_data["z"]),
            point_data["time"]));
    time = point_data["time"];
  }
}

void DataParser::ParseFVToList(GlobalSituation &gs)
{
  for (auto &fv : data["AirCraftList"])
  {

    auto &fv_param = fv["parameters"];

    auto &fv_path = fv["path"];
    FlightPlan path;
    ParsePathFromJSON(fv_path, path);

    string typeName = fv["type"];
    if (!FVNameToType.contains(fv["type"]))
    {
      throw invalid_argument("Unknown vehicle type '" + typeName + "'");
    }
    FVType type = FVNameToType[typeName];
    string navTypeName = fv_param["navigationType"];
    if (!NavMethodNameToType.contains(fv_param["navigationType"]))
    {
      throw invalid_argument("Parsing a FV parameters: the navigation type '" + navTypeName + "' is unknown!");
    }
    NavigationMethods navMethod = NavMethodNameToType[navTypeName];

    switch (type)
    {
    case MASSPOINT:
    {
      double turnRadius;
      if (fv_param.contains("turnRadius"))
      {
        turnRadius = fv_param["turnRadius"];
      }
      else
      {
        turnRadius = -1;
      }
      MassPoint *point = new MassPoint(
          &gs, fv["fv_id"],
          path[0].position.x, path[0].position.y, path[0].position.z,
          fv_param["v_x"], fv_param["v_y"], fv_param["v_z"],
          fv_param["maxAcceleration"], fv_param["k_x"], fv_param["k_v"],
          fv_param["broadcastStep"], fv_param["filterRadius"],
          fv_param["safetyHeight"], fv_param["safetyRadius"],
          navMethod, path, turnRadius);
      gs.addFV(point);
      break;
    }

    case COPTER:
    {
      double turnRadius;
      if (fv_param.contains("turnRadius"))
      {
        turnRadius = fv_param["turnRadius"];
      }
      else
      {
        turnRadius = -1;
      }
      Copter *copter = new Copter(
          &gs, fv["fv_id"],
          path[0].position.x, path[0].position.y, path[0].position.z,
          fv_param["v_x"], fv_param["v_y"], fv_param["v_z"],
          fv_param["inertial_xz"], fv_param["inertial_y"],
          fv_param["k_xz"], fv_param["k_v_xz"], fv_param["k_y"], fv_param["k_v_y"],
          fv_param["maxVelocity_xz"], fv_param["maxVelocity_y"], fv_param["minVelocity_y"],
          fv_param["broadcastStep"], fv_param["filterRadius"],
          fv_param["safetyHeight"], fv_param["safetyRadius"],
          navMethod, path, turnRadius);

      gs.addFV(copter);
      break;
    }

    default:
      throw invalid_argument("The procession of the type '" + typeName + "' is not implemented");
    }
  }
};