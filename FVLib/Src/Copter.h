#pragma once

#include "FV.h"

class Copter : public FV {
public:
  // The constructor
  Copter(GlobalSituation* gs, const string& name,
    double x, double y, double z,
    double speedX, double speedY, double speedZ,
    double inertialXZ, double inertialY, 
    double maxVelocity_xz, double maxVelocity_y, double minVelocity_y,
    double k_xz, double k_v_xz, double k_y, double k_v_y,
    double broadcastStep, double filterRadius, 
    double safetyHeight, double safetyRadius,
    NavigationMethods navType, const FlightPlan& flightPlan,
    double turnRadius = -1);

  // The destructor
  ~Copter();

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

  // Some reasonable turn radius
  double turnRadius;

  // The maximal magnitude of the horizontal velocity
  double maxVelocity_xz;

  // The maximal magnitude of the vertical velocity
  double maxVelocity_y;

  // The minimal magnitude of the vertical velocity
  double minVelocity_y;

  // The inertia coefficient of the horizontal velocity
  double inertialXZ;

  // The inertia coefficient of the vertical velocity
  double inertialY;

  // The proportional coefficient of the horizontal control regulator
  double k_xz;

  // The differential coefficient of the horizontal control regulator
  double k_v_xz;

  // The proportional coefficient of the vertical control regulator
  double k_y;

  // The proportional coefficient of the vertical control regulator
  double k_v_y;
};