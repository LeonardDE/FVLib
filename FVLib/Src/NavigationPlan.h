#pragma once

#include <vector>
using namespace std;

#include "Plane.h"

// Abstract basic class for classes representing navigation plan 
// parts of different kind
class NavigationSegment {
protected:
  // The starting instant of the segment
  double tStart;

  // The final instant of the segment
  double tFinal;

public:
  // Get the state along this part of th navigation plan
  virtual FVState getStateAt(double t) = 0;

  // Getter for the starting time
  virtual double getTStart() {
    return tStart;
  }

  // Getter for the final time
  virtual double getTFinal() {
    return tFinal;
  }
};


// The type for an entire navigation plan
class NavigationPlan {
private:
  // Array of parts of the plan
  vector<NavigationSegment*> segs;

public:
  // The constructor
  NavigationPlan() {}

  // The destructor
  ~NavigationPlan();

  // Checking whether the plan is empty
  bool isEmpty() {
    return segs.size() == 0;
  }

  // Getter for the initial instant of the plan
  // Exception is thrown if the plan is empty
  double getTStart();

  // Getter for the final instant of the plan
  // Exception is thrown if the plan is empty
  double getTFinal();

  // Method for adding a segment to the plan just after the last one
  // exOnError: what to do if the new segment has incorrect time interval
  //    true  - throw an exception
  //    false - return the boolean result
  // Return value:
  //    true  - the segment is added successfully
  //    false - the segment has wrong initial instant
  bool addSegment(NavigationSegment* newSeg, bool exOnError = true);

  // Check whether a time instant belongs to the plan interval
  // Exception is thrown if the plan is empty
  bool doesInstantBelong(double t);
  
  // Get the navigation point of the plan at a given instant
  FVState getStateAt(double t);
};

