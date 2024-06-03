#include "MaterialPoint.h"
#include "GlobalSituation.h"
#include "global.h"

MaterialPoint::MaterialPoint(string name,
	double x, double y, double z,
	double speedX, double speedY, double speedZ,
	double maxAcceleration, double k_x, double k_v,
	double broadcastStep, double radiusFilter, 
	double heightWarn,double radiusWarn,
	GlobalSituation* gs)
{
	this->name = name;
	time = 0;
	curPosition = new Vector3(x, y, z);
	newPosition = new Vector3(x, y, z);
	curVelocity = new Vector3(speedX, speedY, speedZ);
	newVelocity = new Vector3(speedX, speedY, speedZ);
	this->maxAcceleration = maxAcceleration;
	this->k_x = k_x;
	this->k_v = k_v;
	this->radiusFilter = radiusFilter;
	this->radiusWarn = radiusWarn;
	this->heightWarn = heightWarn;


	globalSituation = gs;
	this->broadcastStep = broadcastStep;
}

double MaterialPoint::solveTurnRadius(const Vector3& v1, const Vector3& v2)
{

	return v1.norm() * v2.norm() / (maxAcceleration / 2);
}

void MaterialPoint::next(double h, double end_time)
{
	// Проверка что end_time на промежуток
	int l = 0; // левая граница
	int r = dynamicPath.getPath().size() - 1; // правая граница
	int mid;
	// проверка с концами

	if (end_time < dynamicPath.getPointForIndex(l).arrivalTime ||
		end_time > dynamicPath.getPointForIndex(r).arrivalTime)
	{
		//cout << "Time moment not in path time" << endl;
		return;
	}

	double deltatime = end_time - time;

	doBroadcast();
	while (deltatime > 0) {

		computeWishData(time);

		for (int i = 0; i < 3; i++) {
			double x = (*curPosition)[i];
			double v = (*curVelocity)[i];

			(*newPosition)[i] = x + h * v;
			(*newVelocity)[i] = v + h * (k_x * (x - wishPosition[i]) + k_v * (v - wishVelocity[i]));
			acceleration[i] = (k_x * (x - wishPosition[i]) + k_v * (v - wishVelocity[i]));
		}

		if (deltatime < h)
		{
			h = deltatime;
			deltatime -= h;
		}
		else {
			deltatime -= h;
		}
		time += h;

		swap(curPosition, newPosition);
		swap(curVelocity, newVelocity);
		// Сюда нужно сделать проверку nextBroadcastStep и запись данных радио канал
		doBroadcast();
		
	}
	checkConflict();
}



MaterialPoint::~MaterialPoint()
{
	delete curPosition;
	delete curVelocity;
	delete newPosition;
	delete newVelocity;
}

Plane MaterialPoint::getPlane()
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

string MaterialPoint::getName()
{
	return name;
}

string MaterialPoint::getType()
{
	return type;
}



void MaterialPoint::computeWishData(double time_solve)
{

	int l = 0; // левая граница
	int r = dynamicPath.getPath().size() - 1; // правая граница
	int mid;
	// проверка с концами

	if (time_solve < dynamicPath.getPointForIndex(l).arrivalTime ||
		time_solve > dynamicPath.getPointForIndex(r).arrivalTime)
	{
		cout << "Time moment = "<< time_solve << " not in path time." << " FV_id = " << name << endl;
		return;//exit(0);
	}

	while ((r - l > 1)) {
		mid = (l + r) / 2; // считываем срединный индекс отрезка [l,r]

		if (dynamicPath.getPointForIndex(mid).arrivalTime > time_solve) r = mid; // проверяем, какую часть нужно отбросить
		else l = mid;
	}


	Vector3 wishPos = dynamicPath.getPointForIndex(r).position;
	if (dynamicPath.getPointForIndex(l).type == PointType::DEFAULT) {
		//
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