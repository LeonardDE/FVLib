#pragma once

#include <vector>
using namespace std;

#include "global.h"
#include "Vector3.h"
#include "PathPoint.h"
#include "Plane.h"

// Class for holding a flight plan (a collection of pairs (time,point))
// It fits to store a nominal plan, an actual plan and a short-term plan
class FlightPlan
{
private:
  // The storage of points
  vector<PathPoint> points;

public:
  // Constructors
  FlightPlan() {}
  FlightPlan(const vector<PathPoint> &newPlan) : points(newPlan) {}

  // Getting the size
  size_t size() const
  {
    return points.size();
  }

  // Getting the point by index
  const PathPoint &operator[](int i) const { return points[i]; }

  // Getting the first element
  const PathPoint &first() const
  {
    return points[0];
  }

  // Getting the last element
  const PathPoint &back() const
  {
    return points.back();
  }

  // Clear the plan
  void clear()
  {
    points.clear();
  }

  // Check whether the plan is empty
  bool isEmpty() const
  {
    return points.size() == 0;
  }

  // Get the left bound of the plan time interval
  double getTStart() const
  {
    if (isEmpty())
    {
      throw domain_error("FlightPlan::getTStart: the plan is empty!");
    }
    return points[0].arrivalTime;
  }

  // Get the right bound of the plan time interval
  double getTFinal() const
  {
    if (isEmpty())
    {
      throw domain_error("FlightPlan::getTFinal: the plan is empty!");
    }
    return points.back().arrivalTime;
  }

  // Check whether a time instant belongs to the plan interval
  // Exception is thrown if the plan is empty
  bool doesInstantBelong(double t) const
  {
    return check::GE(t, getTStart()) && check::LE(t, getTFinal());
  }

  // Add a point to the plan
  // exOnError: what to do if the new point is not later than the last point in the plan
  //    true  - throw an exception
  //    false - return the boolean result
  // Return value:
  //    true  - the point is added successfully
  //    false - the point has wrong initial instant
  bool addPoint(const PathPoint &p, bool exOnError = true);

  // Correct this plan with some given plan
  // It is assumed that the time interval of the given plan
  // is strictly inside the time interval of the plan to be corrected
  void CorrectWith(const FlightPlan &addPlan);

  // Get the navigation point of the plan at a given instant
  // t: the time when the state to be computed
  // afterPoint: the index in the array of the point in which interval the time is located
  //   if -1, then the index should be found
  FVState getStateAt(double t, int afterPoint = -1) const;
};
