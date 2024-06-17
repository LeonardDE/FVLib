#pragma once

#include <string>
#include <vector>
using namespace std;

#include "json.h"
using json = nlohmann::json;

#include "Point.h"

class GlobalSituation;

void ParseSolveDataFromJSON(const string& json_name, double& time_step, double& integration_h, int& solve_part_count);

void ParsePathFromJSON(const json& path_data, vector<Point>& path);

void ParseFVToList(const string& json_name, GlobalSituation& gs);
