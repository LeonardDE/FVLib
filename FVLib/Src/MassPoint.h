#pragma once

#include "FV.h"

class MassPoint : public FV {
public:
  MassPoint(string name,
    double x, double y, double z,
    double speedX, double speedY, double speedZ,
    double maxAcceleration, double k_x, double k_v,
    double broadcastStep, double radiusFilter,
    double heightWarn, double radiusWarn, GlobalSituation* gs);
  ~MassPoint();
  FVState getState() override;

  void next(double h, double end_time) override;

  // текущее ускорение 
  Vector3 acceleration;

  Vector3 wishPosition;
  Vector3 wishVelocity;
private:
  Vector3* curPosition;
  Vector3* newPosition;
  Vector3* curVelocity;
  Vector3* newVelocity;

  double maxAcceleration;

  double k_v;
  double k_x;

  double solveTurnRadius(const Vector3& v1, const Vector3& v2) override;

  void computeWishData(double time_solve);
};