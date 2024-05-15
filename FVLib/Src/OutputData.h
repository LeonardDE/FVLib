#pragma once
#include <string>
#include <vector>
#include <fstream>
using namespace std;

#include "json.hpp"
using json = nlohmann::json;

#include "FV.h"


struct OutputObstacle
{

};

struct Parameters
{
	string name;
	string type;
	vector<Point> plan;
};

struct DynamicData
{
	double t;
	double x;
	double y;
	double z;
	double v_x;
	double v_y;
	double v_z;
	bool position_transmitted = false;
	vector<Point> new_plan;

};

struct OutputFV
{
	Parameters parameters;
	vector<DynamicData> dynamic_data;
};

struct OutputJsonData
{
	string version = "1.0";
	vector<OutputObstacle> obstacls;
	vector<OutputFV> FVs;
};

void writeJsonData(const string& file_name, const OutputJsonData& output_json_data);
