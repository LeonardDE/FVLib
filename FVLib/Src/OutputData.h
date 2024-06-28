#pragma once

#include <string>
#include <vector>
#include <fstream>
using namespace std;

#include "json.h"
using json = nlohmann::json;

#include "Vector3.h"
#include "PathPoint.h"
#include "FlightPlan.h"


struct OutputObstacle {
};

struct Parameters {
  string name;
  string type;
  string navType;
  vector<PathPoint> plan;
};

struct DynamicData {
  double t;
  double x;
  double y;
  double z;
};

struct PositionBroadcastData {
  double t;
  double x;
  double y;
  double z;
  double vx;
  double vy;
  double vz;
};

struct PlanBroadcastData {
  double t;
  vector<PathPoint> plan;
};

struct BroadcastData
{
  vector<PositionBroadcastData> positions;
  vector<PlanBroadcastData> plans;
};

struct OutputFV {
  Parameters parameters;
  vector<DynamicData> dynamic_data;
  BroadcastData broadcasts;
};

struct OutputJsonData {
  string version = "2.0";
  vector<OutputObstacle> obstacles;
  vector<OutputFV> FVs;
  double dt;
};

void writeJsonData(const string& file_name, const OutputJsonData& output_json_data);



