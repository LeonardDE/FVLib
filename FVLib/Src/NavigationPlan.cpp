#include <cmath>
#include <stdexcept>
using namespace std;

#include "global.h"
#include "Vector3.h"
#include "Plane.h"
#include "NavigationPlan.h"
#include "RectilinearSegment.h"
#include "CircularSegment.h"

// The destructor
NavigationPlan::~NavigationPlan() {
  for (NavigationSegment* seg : segs) {
    delete seg;
  }
  segs.clear();
}


// Getter for the initial instant of the plan
// Exception is thrown if the plan is empty
double NavigationPlan::getTStart() const {
  if (isEmpty()) {
    throw domain_error("Taking start instant of an empty navigation plan!");
  }
  return segs[0]->getTStart();
}


// Getter for the final instant of the plan
// Exception is thrown if the plan is empty
double NavigationPlan::getTFinal() const {
  if (isEmpty()) {
    throw domain_error("Taking final instant of an empty navigation plan!");
  }
  return segs.back()->getTFinal();
}


// Method for adding a segment to the plan just after the last one
// exOnError: what to do if the new segment has incorrect time interval
//    true  - throw an exception
//    false - return the boolean result
// Return value:
//    true  - the segment is added successfully
//    false - the segment has wrong initial instant
bool NavigationPlan::addSegment(NavigationSegment* newSeg, bool exOnError) {
  if (isEmpty()) {
    segs.push_back(newSeg);
    return true;
  }
  else {
    if (check::NE(segs.back()->getTFinal(), newSeg->getTStart())) {
      if (exOnError) {
        throw invalid_argument("Adding a segment to a navigation plan: the new segment is not adjacent to the final one!");
      }
      else {
        return false;
      }
    }
    else {
      segs.push_back(newSeg);
      return true;
    }
  }
}


// Check whether a time instant belongs to the plan interval
// Exception is thrown if the plan is empty
bool NavigationPlan::doesInstantBelong(double t) {
  if (isEmpty()) {
    throw domain_error("Taking final instant of an empty navigation plan!");
  }
  return check::LE(getTStart(), t) && check::LE(t, getTFinal());
}


// Get the navigation point of the plan at a given instant
FVState NavigationPlan::getStateAt(double t) {
  if (!doesInstantBelong(t)) {
    throw domain_error("Taking the navigation point at an instant outside the time interval of the plan!");
  }

  // Check whether the instant belongs to the final segment
  if (check::GE(t, segs.back()->getTStart())) {
    return segs.back()->getStateAt(t);
  }

  // Binary search for the segment containing the given instant:
  //   l is the index of a segment whose initial instant is less than or equal to t
  //   r is the index of a segment whose initial instant is greater than t
  int l = 0, r = segs.size() - 1;
  while (l + 1 < r) {
    int m = (l + r) / 2;
    if (check::GE(t, segs[m]->getTStart())) {
      l = m;
    }
    else {
      r = m;
    }
  }

  return segs[l]->getStateAt(t);
}


// Create naive navigation plan
void NavigationPlan::CreateNaivePlan(vector<PathPoint> flightPlan) {
  clear();
  for (int i = 1, l = flightPlan.size(); i < l; i++) {
    Vector3 vel = (flightPlan[i].position - flightPlan[i - 1].position) /
      (flightPlan[i].arrivalTime - flightPlan[i - 1].arrivalTime);
    addSegment(new RectilinearSegment(
      flightPlan[i - 1].arrivalTime, flightPlan[i - 1].position, vel,
      flightPlan[i].arrivalTime, flightPlan[i].position, vel));

  }
}

// Create navigation plan with impossible circular turns
void NavigationPlan::CreateArcTurnPlan(vector<PathPoint> flightPlan, double turnRadius) {
  clear();
  Vector3 prevPoint = flightPlan[0].position;
  double prevTime = flightPlan[0].arrivalTime;

  int l = flightPlan.size();
  for (int i = 2; i < l; i++) {
    Vector3 AB = flightPlan[i - 1].position - prevPoint;
    Vector3 BC = flightPlan[i].position - flightPlan[i - 1].position;
    Vector3 velAB = AB / (flightPlan[i - 1].arrivalTime - prevTime);
    Vector3 velBC = BC / (flightPlan[i].arrivalTime - flightPlan[i - 1].arrivalTime);
    double cosAlpha = getCosBetweenVectors(AB, BC);

    if (check::EQ(cosAlpha, 1)) {
      // The case of two (almost) parallel neighbor segments - no turn
      addSegment(new RectilinearSegment(prevTime, prevPoint, velAB,
        flightPlan[i - 1].arrivalTime, flightPlan[i - 1].position, velAB));
      prevPoint = flightPlan[i - 1].position;
      prevTime = flightPlan[i - 1].arrivalTime;
    }
    else {
      // The case of neighbor segments having a significant angle between them - need a turn 
      double tanAlpha2 = sqrt((1 - cosAlpha) / (1 + cosAlpha));
      double h = turnRadius * tanAlpha2;

      Vector3 AB0 = AB.getNormVector();
      Vector3 BC0 = BC.getNormVector();
      Vector3 pos1 = flightPlan[i - 1].position - h * AB0;
      Vector3 pos2 = flightPlan[i - 1].position + h * BC0;

      Vector3 bisec = (BC0 - AB0).getNormVector();
      Vector3 center = flightPlan[i - 1].position + (turnRadius * sqrt(1 + pow(tanAlpha2, 2))) * bisec;
      double t1 = flightPlan[i - 1].arrivalTime - h / velAB.norm();
      double t2 = flightPlan[i - 1].arrivalTime + h / velBC.norm();

      addSegment(new CircularSegment(t1, pos1, velAB, t2, pos2, velBC, center));

      prevPoint = pos2;
      prevTime = t2;
    }
  }

  Vector3 lastVel = (flightPlan[l - 1].position - flightPlan[l - 2].position) /
    (flightPlan[l - 1].arrivalTime = flightPlan[l - 2].arrivalTime);
  addSegment(new RectilinearSegment(prevTime, prevPoint, lastVel,
    flightPlan[l - 1].arrivalTime, flightPlan[l - 1].position, lastVel));
}


// Structure to convert a string to the NavigationMethods enum
map<string, NavigationMethods> NavMethodNameToType = {
  { "Naive"    , NAIVE    },
  { "Circular" , CIRCULAR }
};


// Structure to convert a NavigationMethods enum to the string
map<NavigationMethods, string> NavMethodTypeToName = {
  { NAIVE    , "Naive"    },
  { CIRCULAR , "Circular" }
};