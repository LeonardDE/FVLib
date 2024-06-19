#pragma once

#include "FlightPlan.h"

class DecisionMaker {
public:
  static void mergeShortPlans(const FlightPlan& arr1, const FlightPlan& arr2, FlightPlan& res1, FlightPlan& res2);
};

