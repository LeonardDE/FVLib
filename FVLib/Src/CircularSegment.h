#pragma once

#include "NavigationPlan.h"

// Class for a segment of a navigation plan with uniformly variable motion along an arc of a circle
// with uniformly variable velocity changing independently
class CircularSegment : public NavigationSegment {
protected:
  // Nominal change of the velocity
  Vector3 nomAccel;

  // Center of the arc
  Vector3 arcCenter;

  // Axis of rotation along the arc
  Vector3 axis;

  // Turn angle
  double turnAngle;

public:
  CircularSegment(double t1, const Vector3& pos1, const Vector3& vel1,
    double t2, const Vector3& pos2, const Vector3& vel2,
    const Vector3& center);

  // Get the state along this part of the navigation plan
  FVState getStateAt(double t) override;
};