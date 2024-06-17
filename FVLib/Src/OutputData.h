#pragma once

#include <string>
#include <vector>
#include <fstream>
using namespace std;

#include "json.h"
using json = nlohmann::json;

#include "Vector3.h"
#include "Point.h"


struct OutputObstacle {
};

struct Parameters {
  string name;
  string type;
  vector<Point> plan;
};

struct DynamicData {
  double t;
  double x;
  double y;
  double z;
};

struct BroadcastData {
  double t;
  string type;

  double x = 0;
  double y = 0;
  double z = 0;
  vector<Point> plan;
};

struct OutputFV {
  Parameters parameters;
  vector<DynamicData> dynamic_data;
  vector<BroadcastData> broadcasts;
};

struct OutputJsonData {
  string version = "1.0";
  vector<OutputObstacle> obstacles;
  vector<OutputFV> FVs;
  double dt;
};

void writeJsonData(const string& file_name, const OutputJsonData& output_json_data);


