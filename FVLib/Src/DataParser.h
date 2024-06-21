#pragma once

#include <string>
#include <vector>
using namespace std;

#include "json.h"
using json = nlohmann::json;

#include "FlightPlan.h"

// Forward declaration of the type of the structure for keeping the current air-space situation
class GlobalSituation;


// Method for parsing integration configuration data from the input file
void ParseSolveDataFromJSON(const string& json_name, double& time_step, double& integration_h, int& solve_part_count);


// Method for parsing a flight plan from the input file
void ParsePathFromJSON(const json& path_data, FlightPlan& path);


// Method for parsing the list of FVs from the input file
void ParseFVToList(const string& json_name, GlobalSituation& gs);
