#pragma once
#include <string>
#include <vector>
#include "json.hpp"
#include "Point.h"
using namespace std;
using json = nlohmann::json;

class GlobalSituation;

void ParseSolveDataFromJSON(string json_name, double& solve_time, double& integration_h, int& solve_part_count);

void ParsePathFromJSON(const json& path_data, vector<Point>& path);

void ParseFVToList(string json_name, GlobalSituation& gs);
