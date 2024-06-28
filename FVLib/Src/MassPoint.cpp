#include "global.h"
#include "Plane.h"
#include "GlobalSituation.h"
#include "MassPoint.h"

// The constructor
MassPoint::MassPoint(GlobalSituation* gs, const string& name,
  double x, double y, double z,
  double speedX, double speedY, double speedZ,
  double maxAcceleration, double k_x, double k_v,
  double broadcastStep, double filterRadius,
  double safetyHeight, double safetyRadius,
  NavigationMethods navType, const FlightPlan& flightPlan,
  double turnRadius
) {

  type = MASSPOINT;
  this->name = name;
  time = flightPlan[0].arrivalTime;
  curPosition = new Vector3(x, y, z);
  newPosition = new Vector3(x, y, z);
  curVelocity = new Vector3(speedX, speedY, speedZ);
  newVelocity = new Vector3(speedX, speedY, speedZ);
  this->maxAcceleration = maxAcceleration;
  this->k_x = k_x;
  this->k_v = k_v;
  this->filterRadius = filterRadius;
  this->safetyRadius = safetyRadius;
  this->safetyHeight = safetyHeight;
  this->navType = navType;

  globalSituation = gs;

  // Setting the turn radius 
  if (turnRadius > 0) {
    this->turnRadius = turnRadius;
  }
  else {
    // If not given, the turn radius is computed for the velocity,
    // which can be achieved during 5 seconds with the maximal acceleration,
    // and for the lateral acceleration equal to the half of the maximal one
    this->turnRadius = 50 * maxAcceleration; // pow(5 * maxAcceleration, 2) / (0.5 * maxAcceleration)
  }

  basePath = flightPlan;
  currentPath = flightPlan;
  navigationPath.CreatePlan(currentPath, navType, turnRadius);

  this->broadcastStep = broadcastStep;
  nextBroadcastInstant = currentPath[0].arrivalTime;
}

void MassPoint::next(double h, double end_time) {
  double dt = end_time - time;
  FVState aimingPoint;
  Vector3 acceleration;
  int i;

  doBroadcast();

  while (dt > 0) {

    // Producing the control acceleration with saturation
    aimingPoint = navigationPath.getStateAt(time);
    for (i = 0; i < 3; i++) {
      acceleration[i] = k_x * ((*curPosition)[i] - aimingPoint.position[i]) +
        k_v * ((*curVelocity)[i] - aimingPoint.velocity[i]);
    }
    double accelNorm = acceleration.norm();
    if (accelNorm > maxAcceleration) {
      acceleration *= maxAcceleration / accelNorm;
    }

    // Integrating the motion
    for (i = 0; i < 3; i++) {
      (*newPosition)[i] = (*curPosition)[i] + h * (*curVelocity)[i];
      (*newVelocity)[i] = (*curVelocity)[i] + h * acceleration[i];
    }

    // Increasing time
    if (dt < h) {
      h = dt;
      dt -= h;
    }
    else {
      dt -= h;
    }
    time += h;

    // Put the new values into the current storage
    swap(curPosition, newPosition);
    swap(curVelocity, newVelocity);

    // Do broadcasts if necessary
    doBroadcast();
  }

}



// Destructor
MassPoint::~MassPoint() {
  delete curPosition;
  delete curVelocity;
  delete newPosition;
  delete newVelocity;
}


// Method to take data of the current position for computations
FVState MassPoint::getState() const {
  FVState state;
  state.position = *curPosition;
  state.velocity = *curVelocity;

  return state;
}


// Method to take data of the current position for output
DynamicData MassPoint::getOutputPosition() const {
  DynamicData res;
  res.t = time;
  res.x = (*curPosition)[0];
  res.y = (*curPosition)[1];
  res.z = (*curPosition)[2];

  return res;
}

// Method to take data of the current position for final writing
FVOutputState MassPoint::getOutputState() const {
  FVOutputState plane;

  plane.x = (*curPosition)[0];
  plane.y = (*curPosition)[1];
  plane.z = (*curPosition)[2];
  plane.speedX = (*curVelocity)[0];
  plane.speedY = (*curVelocity)[1];
  plane.speedZ = (*curVelocity)[2];
  plane.speed = sqrt(pow(plane.speedX, 2) + pow(plane.speedY, 2) + pow(plane.speedZ, 2));
  plane.name = name;
  plane.type = FVTypeToName[type];

  return plane;
}


