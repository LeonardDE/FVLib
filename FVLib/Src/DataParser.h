#pragma once

#include <string>
#include <vector>
using namespace std;

#include "json.h"
using json = nlohmann::json;

#include "FlightPlan.h"

// Forward declaration of the type of the structure for keeping the current air-space situation
class GlobalSituation;

class DataParser
{
private:
  json data;

  // Method for parsing a flight plan from the input file
  void ParsePathFromJSON(const json& path_data, FlightPlan &path);

public:
  // The current version of the configuration
  static const string version; 

  // The constructor
  DataParser(const string &json_name);

  // Method for parsing integration configuration data from the input file
  void ParseSolveDataFromJSON(double &time_step, int &solve_part_count);

  // Method for parsing the list of FVs from the input file
  void ParseFVToList(GlobalSituation &gs);
};
