#pragma once

#include "FV.h"

class Copter : public FV {
public:
  Copter(const string& name, double x, double y, double z,
    double speedX, double speedY, double speedZ,
    double inertialXZ, double inertialY, double k_x, double k_v,
    double maxVelocity_xz, double maxVelocity_y, double minVelocity_y,
    double radiusFilter, double broadcastStep,
    double heightWarn, double radiusWarn, GlobalSituation* gs);
  ~Copter();
  FVOutputState getOutputState() override;
  void next(double h, double end_time) override;



  Vector3 wishVelocity;
  Vector3 wishPosition;
private:
  double inertialXZ;
  double inertialY;
  double k_x;
  double k_v;
  Vector3* curPosition;
  Vector3* newPosition;
  Vector3* curVelocity;
  Vector3* newVelocity;

  double maxVelocity_xz;
  double maxVelocity_y;
  double minVelocity_y;

  double solveTurnRadius(const Vector3& v1, const Vector3& v2) override;

  void computeWishData(double time_solve);
};