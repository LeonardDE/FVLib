#include <stdexcept>
using namespace std;

#include "global.h"
#include "Plane.h"
#include "NavigationPlan.h"

// The destructor
NavigationPlan::~NavigationPlan() {
  for (NavigationSegment* seg : segs) {
    delete seg;
  }
  segs.clear();
}


// Getter for the initial instant of the plan
// Exception is thrown if the plan is empty
double NavigationPlan::getTStart() {
  if (isEmpty()) {
    throw domain_error("Taking start instant of an empty navigation plan!");
  }
  return segs[0]->getTStart();
}


// Getter for the final instant of the plan
// Exception is thrown if the plan is empty
double NavigationPlan::getTFinal() {
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
bool NavigationPlan::addSegment(NavigationSegment* newSeg, bool exOnError = true) {
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