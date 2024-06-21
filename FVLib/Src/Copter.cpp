#include "Copter.h"
#include "GlobalSituation.h"


// The constructor
Copter::Copter(GlobalSituation* gs, const string& name,
  double x, double y, double z,
  double speedX, double speedY, double speedZ,
  double inertialXZ, double inertialY,
  double maxVelocity_xz, double maxVelocity_y, double minVelocity_y,
  double k_xz, double k_v_xz, double k_y, double k_v_y,
  double broadcastStep, double filterRadius,
  double safetyHeight, double safetyRadius,
  NavigationMethods navType, const FlightPlan& flightPlan,
  double turnRadius
) {
  this->name = name;
  time = 0;

  curPosition = new Vector3{ x,y,z };
  newPosition = new Vector3{ x,y,z };
  curVelocity = new Vector3{ speedX,speedY,speedZ };
  newVelocity = new Vector3{ speedX,speedY,speedZ };

  this->navType = navType;

  this->maxVelocity_xz = maxVelocity_xz;
  this->maxVelocity_y = maxVelocity_y;
  this->minVelocity_y = minVelocity_y;

  this->inertialXZ = inertialXZ;
  this->inertialY = inertialY;

  this->k_xz = k_xz;
  this->k_v_xz = k_v_xz;
  this->k_y = k_y;
  this->k_v_y = k_v_y;

  this->filterRadius = filterRadius;
  this->safetyRadius = safetyRadius;
  this->safetyHeight = safetyHeight;

  globalSituation = gs;
  this->broadcastStep = broadcastStep;

  // Setting the turn radius
  if (turnRadius > 0) {
    this->turnRadius = turnRadius;
  }
  else {
    // Some maximal acceleration the FV can achieve
    double maxAccel = 4 * max(2 * maxVelocity_xz / inertialXZ, (maxVelocity_y - minVelocity_y) / inertialY);

    // Some maximal velocity the FV can achieve
    double maxVel = max(max(maxVelocity_xz, fabs(maxVelocity_y)), fabs(minVelocity_y));

    turnRadius = pow(maxVel, 2) / maxAccel;
  }

  basePath = flightPlan;
  currentPath = flightPlan;
  navigationPath.CreatePlan(currentPath, navType, turnRadius);
}

// The destructor
Copter::~Copter() {
  delete curPosition;
  delete curVelocity;
  delete newPosition;
  delete newVelocity;
}


// Integration of the motion
void Copter::next(double h, double end_time) {
  double dt = end_time - time;
  FVState aimingState;
  Vector3 control;
  double horLen;
  double vertControl;
  double k;

  doBroadcast();

  while (dt > 0) {
    aimingState = navigationPath.getStateAt(time);

    // Generating the horizontal control
    control[0] = k_xz * ((*curPosition)[0] - aimingState.position[0]) +
      k_v_xz * ((*curVelocity)[0] - aimingState.velocity[0]);
    control[1] = 0;
    control[2] = k_xz * ((*curPosition)[2] - aimingState.position[2]) +
      k_v_xz * ((*curVelocity)[2] - aimingState.velocity[2]);
    horLen = control.norm();
    if (horLen > maxVelocity_xz) {
      control *= maxVelocity_xz / horLen;
    }

    // Generating the vertical control
    vertControl = k_y * ((*curPosition)[1] - aimingState.position[1]) +
      k_v_xz * ((*curVelocity)[1] - aimingState.velocity[1]);
    if (vertControl > maxVelocity_y) {
      vertControl = maxVelocity_y;
    }
    else if (vertControl < minVelocity_y) {
      vertControl = minVelocity_y;
    }
    control[1] = vertControl;

    for (int i = 0; i < 3; i++) {
      k = i == 1 ? inertialXZ : inertialXZ;

      (*newPosition)[i] = (*curPosition)[i] + h * (*curVelocity)[i];
      (*newVelocity)[i] = (control[i] - (*curVelocity)[i]) / k;
    }

    time += h;
    if (dt < h) {
      h = dt;
    }
    else {
      dt -= h;
    }

    swap(curPosition, newPosition);
    swap(curVelocity, newVelocity);

    doBroadcast();
  }

  time = end_time;

}



// Method to take data of the current position for computations
FVState Copter::getState() const {
  FVState state;
  state.position = *curPosition;
  state.velocity = *curVelocity;

  return state;
}


// Method to take data of the current position for final writing
FVOutputState Copter::getOutputState() const {
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


