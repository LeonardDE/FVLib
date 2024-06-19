#include <stdexcept>
using namespace std;

#include "global.h"
#include "RectilinearSegment.h"

// The constructor
RectilinearSegment::RectilinearSegment(double t1, const Vector3& pos1, const Vector3& vel1,
  double t2, const Vector3& pos2, const Vector3& vel2) {
  if (check::EQ(t1, t2)) {
    throw invalid_argument("RectilinearSegment constructor: equal time instants are given!");
  }

  tStart = t1;
  posStart = pos1;
  velStart = vel1;

  tFinal = t2;
  posFinal = pos2;
  velFinal = vel2;

  double dt = t2 - t1;
  nomVelocity = (pos2 - pos1) / dt;
  nomAccel = (vel2 - vel1) / dt;
}

// Get the state along this part of the navigation plan
FVState RectilinearSegment::getStateAt(double t) {
  if (check::LT(t, tStart) || check::GT(t, tFinal)) {
    throw domain_error("RectilinearSegment::getStateAt: time outside the segment interval!");
  }

  double dt = t - tStart;

  FVState state;
  state.position = posStart + dt * nomVelocity;
  state.velocity = velStart + dt * nomAccel;

  return state;
}
