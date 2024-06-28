#include <fstream>
using namespace std;

#include "Vector3.h"
#include "PathPoint.h"
#include "FlightPlan.h"
#include "OutputData.h"



NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Vector3, x, y, z);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathPoint, position, arrivalTime);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Parameters, name, type, navType, plan);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DynamicData, t, x, y, z);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PositionBroadcastData, t, x, y, z, vx, vy, vx);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PlanBroadcastData, t, plan);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BroadcastData, positions, plans);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OutputFV, parameters, dynamic_data, broadcasts);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OutputJsonData, version, dt, FVs);



void writeJsonData(const string& file_name, const OutputJsonData& output_json_data) {
  json j = output_json_data;
  ofstream f(file_name);
  f << setw(3) << j << endl;
  f.close();
};
