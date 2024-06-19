#include <vector>
using namespace std;

#include "global.h"
#include "FlightPlan.h"

// Add a point to the plan
// exOnError: what to do if the new point is not later than the last point in the plan
//    true  - throw an exception
//    false - return the boolean result
// Return value:
//    true  - the point is added successfully
//    false - the point has wrong initial instant
bool FlightPlan::addPoint(const PathPoint& p, bool exOnError = true) {
  if (!isEmpty() && check::LE(p.arrivalTime, points.back().arrivalTime)) {
    if (exOnError) {
      throw invalid_argument("FlightPlan::addPoint: the given point is not after the last point of the plan!");
    }
    else {
      return false;
    }
  }

  points.push_back(p);
  return true;
}

// Correct this plan with some given plan
// It is assumed that the time interval of the given plan 
// is strictly inside the time interval of the plan to be corrected
void FlightPlan::CorrectWith(const FlightPlan& addPlan) {
  if (addPlan.isEmpty()) {
    throw domain_error("FlightPlan::CorrectWith: the given plan is empty, nothing to correct!");
  }

  if (check::LE(addPlan.getTStart(), getTStart()) &&
    check::GE(addPlan.getTFinal(), getTFinal())) {
    throw domain_error("FlightPlan::CorrectWith: the time interval of the given plan is not strictly within the time interval of the plan to corrected!");
  }

  vector<PathPoint> resPlan;
  int cntOrig = 0, cntAdd = 0;

  // Adding the initial part of the original plan
  while (check::LT(points[cntOrig].arrivalTime, addPlan[0].arrivalTime)) {
    resPlan.push_back(points[cntOrig]);
    cntOrig++;
  }

  // Adding the corrected plan
  for (const PathPoint& p : addPlan.points) {
    resPlan.push_back(p);
  }

  // Passing the points of the original plan
  while (check::LE(points[cntOrig].arrivalTime, addPlan.points.back().arrivalTime)) {
    cntOrig++;
  }

  // Adding the tail of the original plan
  while (cntOrig < points.size()) {
    resPlan.push_back(points[cntOrig]);
    cntOrig++;
  }

  points = resPlan;
}

