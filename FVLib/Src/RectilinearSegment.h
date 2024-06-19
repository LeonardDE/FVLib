#pragma once

#include "NavigationPlan.h"

// Class for a segment of a navigation plan with rectilinear uniformly variable motion
class RectilinearSegment : public NavigationSegment {
protected:
  // Nominal change of the position
  Vector3 nomVelocity;

  // Nominal change of the velocity
  Vector3 nomAccel;

public:
  RectilinearSegment(double t1, const Vector3& pos1, const Vector3& vel1,
    double t2, const Vector3& pos2, const Vector3& vel2);

  // Get the state along this part of the navigation plan
  FVState getStateAt(double t) override;
};