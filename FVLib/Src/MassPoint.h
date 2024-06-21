#pragma once

#include "FV.h"

class MassPoint : public FV {
public:
  // The constructor
  MassPoint(GlobalSituation* gs, const string& name,
    double x, double y, double z,
    double speedX, double speedY, double speedZ,
    double maxAcceleration, double k_x, double k_v,
    double broadcastStep, double radiusFilter,
    double safetyHeight, double safetyRadius,
    NavigationMethods navType, const FlightPlan& flightPlan,
    double turnRadius = -1);

  // The destructor
  ~MassPoint();

  // Method to take data of the current position for computations
  FVState getState() const override;

  // Method to take data of the current position for final writing
  FVOutputState getOutputState() const override;

  // Method to integrate motion of the FV up to the given time with the given time step
  void next(double h, double end_time) override;

private:
  // The current value of the position
  Vector3* curPosition;

  // The storage for the position at the next time instant
  Vector3* newPosition;

  // The current value of the velocity
  Vector3* curVelocity;

  // The storage for the velocity at the next time instant
  Vector3* newVelocity;

  // Constraint for the maximal acceleration
  double maxAcceleration;

  // Some reasonable turn radius
  double turnRadius;

  // The regulator proportional coefficient
  double k_v;

  // The regulator differential coefficient
  double k_x;
};
