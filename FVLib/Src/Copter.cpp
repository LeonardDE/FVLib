#include "Copter.h"
#include "GlobalSituation.h"


Copter::Copter(const string& name, double x, double y, double z,
	double speedX, double speedY, double speedZ,
	double inertialXZ, double inertialY, double k_x, double k_v,
	double maxVelocity_xz, double maxVelocity_y, double minVelocity_y,
	double broadcastStep, double radiusFilter,
	double heightWarn, double radiusWarn, GlobalSituation* gs)
{
	this->name = name;
	this->type = 1;
	time = 0;

	this->inertialXZ = inertialXZ;
	this->inertialY = inertialY;

	curPosition = new Vector3{ x,y,z };
	newPosition = new Vector3{ x,y,z };
	curVelocity = new Vector3{ speedX,speedY,speedZ };
	newVelocity = new Vector3{ speedX,speedY,speedZ };

	this->k_x = k_x;
	this->k_v = k_v;
	this->maxVelocity_xz = maxVelocity_xz;
	this->maxVelocity_y = maxVelocity_y;
	this->minVelocity_y = minVelocity_y;

	this->radiusFilter = radiusFilter;
	this->radiusWarn = radiusWarn;
	this->heightWarn = heightWarn;


	globalSituation = gs;
	this->broadcastStep = broadcastStep;
}

Copter::~Copter()
{
	delete curPosition;
	delete curVelocity;
	delete newPosition;
	delete newVelocity;
}

void Copter::next(double h, double end_time)
{
	double deltatime = end_time - time;
	while (deltatime > 0) {


		double x = (*curPosition)[0];
		double v = (*curVelocity)[0];

		(*newPosition)[0] = x + h * v;
		(*newVelocity)[0] = v + h * ((k_v * wishVelocity.x + k_x * wishPosition.x) - v) / inertialXZ;

		x = (*curPosition)[1];
		v = (*curVelocity)[1];

		(*newPosition)[1] = x + h * v;
		(*newVelocity)[1] = v + h * ((k_v * wishVelocity.y + k_x * wishPosition.x) - v) / inertialY;

		x = (*curPosition)[2];
		v = (*curVelocity)[2];

		(*newPosition)[2] = x + h * v;
		(*newVelocity)[2] = v + h * ((k_v * wishVelocity.z + k_x * wishPosition.x) - v) / inertialXZ;


		time += h;
		if (deltatime < h)
		{
			h = deltatime;
		}
		else
		{
			deltatime -= h;
		}
		swap(curPosition, newPosition);
		swap(curVelocity, newVelocity);
	}
	time = end_time;
	swap(curPosition, newPosition);
	swap(curVelocity, newVelocity);

}


double Copter::solveTurnRadius(const Vector3& v1, const Vector3& v2)
{
	// рср врн рн асдер
	return 1;
}

void Copter::computeWishData(double time_solve)
{
	int l = 0;
	int r = (int)dynamicPath.getPath().size() - 1;
	int mid;


	if (time_solve < dynamicPath.getPointForIndex(l).arrivalTime ||
		time_solve > dynamicPath.getPointForIndex(r).arrivalTime)
	{
		cout << "Time moment not in path time" << endl;
		exit(0);
	}

	while ((r - l > 1)) {
		mid = (l + r) / 2;

		if (dynamicPath.getPointForIndex(mid).arrivalTime > time_solve) r = mid;
		else l = mid;
	}


	Vector3 wishPos = dynamicPath.getPointForIndex(r).position;
	if (dynamicPath.getPointForIndex(l).type == PointType::DEFAULT) {

		if (r == 0)
		{
			wishVelocity = (dynamicPath.getPointForIndex(r).position - *curPosition) /
				(dynamicPath.getPointForIndex(r).arrivalTime - time_solve);
		}
		else {
			wishVelocity = (dynamicPath.getPointForIndex(r).position - dynamicPath.getPointForIndex(l).position) /
				(dynamicPath.getPointForIndex(r).arrivalTime - dynamicPath.getPointForIndex(l).arrivalTime);
		}

		wishPos = dynamicPath.getPointForIndex(l).position
			+ wishVelocity * (time_solve - dynamicPath.getPointForIndex(l).arrivalTime);

	}
	else if (dynamicPath.getPointForIndex(l).type == PointType::START_TURN)
	{

		TurnData turn_data = dynamicPath.getPointForIndex(l).turnData;
		double deltaT = time_solve - dynamicPath.getPointForIndex(l).arrivalTime;

		double w = turn_data.angularVelocity + deltaT * turn_data.angularAcceleration;

		Vector3 v = crossProduct(turn_data.axis * w, wishPosition - turn_data.axisPoint);

		wishVelocity = v;

		wishPos = dynamicPath.getPointForIndex(l).position;

		wishPos.rotate(
			turn_data.axis,
			turn_data.axisPoint,

			turn_data.angularVelocity_0 * deltaT);


	}

	wishPosition = wishPos;
}

Plane Copter::getPlane()
{
	Plane plane = Plane();
	plane.x = (*curPosition)[0];
	plane.y = (*curPosition)[1];
	plane.z = (*curPosition)[2];
	plane.speedX = (*curVelocity)[0];
	plane.speedY = (*curVelocity)[1];
	plane.speedZ = (*curVelocity)[2];
	plane.speed = sqrt(plane.speedX * plane.speedX +
		plane.speedY * plane.speedY +
		plane.speedZ * plane.speedZ);
	plane.name = name;
	plane.type = type;
	return plane;
}
