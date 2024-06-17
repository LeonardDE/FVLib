#include <fstream>
using namespace std;

#include "OutputData.h"

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Vector3, x, y, z);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point, position, arrivalTime);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Parameters, name,type, plan);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DynamicData, t, x, y,z);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BroadcastData, t, type,x,y,z,plan);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OutputFV, parameters,dynamic_data,broadcasts);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OutputJsonData, version, dt, FVs);



void writeJsonData(const string& file_name, const OutputJsonData& output_json_data)
{
	json j = output_json_data;
	//cout << j.dump(1)<< endl;
	ofstream f(file_name);
	f << setw(3) << j << endl;
	f.close();
};
