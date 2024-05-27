#include "OutputData.h"
#include <fstream>
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Vector3, x, y, z);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point, position, arrivalTime);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Parameters, name,type, plan);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DynamicData, t, x, y,z, v_x, v_y, v_z, position_transmitted, new_plan);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OutputFV, parameters,dynamic_data);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OutputJsonData, FVs, version);


void writeJsonData(const string& file_name, const OutputJsonData& output_json_data)
{
	json j = output_json_data;
	//cout << j.dump(1)<< endl;
	ofstream f(file_name);
	f << setw(4) << j << endl;

};
