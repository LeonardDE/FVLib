#include <cmath>
#include <stdexcept>
using namespace std;

#include "global.h"
#include "Vector3.h"
#include "CircularSegment.h"

CircularSegment::CircularSegment(double t1, const Vector3& pos1, const Vector3& vel1,
  double t2, const Vector3& pos2, const Vector3& vel2, const Vector3& center) {

  if (check::NE(distance(pos1, center), distance(pos2, center))) {
    throw invalid_argument("CircularSegment constructor: distances from the center to the positions are not equal!");
  }

  tStart = t1;
  posStart = pos1;
  velStart = vel1;
  
  tFinal = t2;
  posFinal = pos2;
  velFinal = vel2;

  arcCenter = center;
  
  nomAccel = (vel2 - vel1) / (t2 - t1);
  Vector3 v1 = pos1 - center;
  Vector3 v2 = pos2 - center;
  axis = crossProduct(v1, v2).getNormVector();
  turnAngle = acos(getCosBetweenVectors(v1, v2));
}

FVState CircularSegment::getStateAt(double t) {
  if (check::LT(t, tStart) || check::GT(t, tFinal)) {
    throw domain_error("RectilinearSegment::getStateAt: time outside the segment interval!");
  }

  double dt = t - tStart;

  FVState state;
  state.velocity = velStart + dt * nomAccel;
  state.position = posStart.getRotatedVector(axis, arcCenter, dt * turnAngle / (tFinal - tStart));

  return state;
}
