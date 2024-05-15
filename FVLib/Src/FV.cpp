#include "FV.h"

MaterialPoint::MaterialPoint(string name,
	double x, double y, double z,
	double speedX, double speedY, double speedZ,
	double maxAcceleration, double k_x, double k_v)
{
	this->name = name;
	this->type = 0;
	time = 0;
	curPosition = new Vector3( x,y,z );
	newPosition = new Vector3( x,y,z );
	curVelocity = new Vector3( speedX,speedY,speedZ );
	newVelocity = new Vector3( speedX,speedY,speedZ );
	this->maxAcceleration = maxAcceleration;
	this->k_x = k_x;
	this->k_v = k_v;
}
// переделать передачу массивая
void FV::setPath(vector<Point> path)
{
	
	this->basePath = path;
	double turnAcceleration = this->maxAcceleration / 2;

	Path new_path = Path();

	if (path.size() > 2)
	{
		for (int i = 0; i < path.size() - 2; i++)
		{
			Vector3 AB = path[i+1].position - path[i ].position;
			Vector3 BC = path[i + 2].position - path[i + 1].position;
			double cos_a = getCosBetweenVectors(AB * (-1), BC);
			
			if (abs(cos_a) < 0.99)
			{
				Vector3 vel_1 = AB / (path[i + 1].arrivalTime - path[i].arrivalTime);
				Vector3 vel_2 = BC / (path[i + 2].arrivalTime - path[i + 1].arrivalTime);

				double r = vel_1.norm() * vel_1.norm() / turnAcceleration;
				double l = r * sqrt((1 + cos_a) / (1 - cos_a));

				Vector3 turnPosition_1 = path[i ].position + vel_1 * (AB.norm() - l) / vel_1.norm();
				double turnTime_1 = path[i + 1].arrivalTime - l / vel_1.norm();

				Vector3 turnPosition_2 = path[i + 1].position + vel_2 * l / vel_2.norm();
				double turnTime_2 = path[i + 1].arrivalTime + l / vel_2.norm();


				double w1 = vel_1.norm() / r;
				double w2 = vel_2.norm() / r;
				double eps = (w2 - w1) / (turnTime_2 - turnTime_1);

				double w_0 = acos(-cos_a) / (turnTime_2 - turnTime_1);

				Vector3 BO = ((AB.getNormVector() * (-1) + BC.getNormVector()) / 2).getNormVector() * r * sqrt(2/(1-cos_a));
				Vector3 axisPoint = path[i + 1].position + BO;

				Vector3 axis = crossProduct(AB , BC).getNormVector();
				
				TurnData data_1 = TurnData(w1,eps,w_0,axis,axisPoint);

				new_path.addNewPoint(PathPoint(path[i]));
				new_path.addNewPoint(PathPoint(turnPosition_1,turnTime_1,PointType::START_TURN,data_1));
				path[i + 1] = Point(turnPosition_2, turnTime_2);
			}
			else
			{
				new_path.addNewPoint(PathPoint(path[i]));
			}
		}
		new_path.addNewPoint(path[path.size() - 2]);
		new_path.addNewPoint(path[path.size() - 1]);
	}

	dynamicPath = new_path;
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


	while (deltatime > 0) {

		computeWishData(time);
		
		for (int i = 0; i < 3; i++) {
				double x = (*curPosition)[i];
				double v = (*curVelocity)[i];

				(*newPosition)[i] = x + h * v;
				(*newVelocity)[i] = v + h * (k_x * (x -  wishPosition[i]) + k_v * (v - wishVelocity[i]));
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
	}

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

int32_t MaterialPoint::getType() 
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
		cout << "Time moment not in path time" << endl;
		exit(0);
	}

	while ((r - l > 1) ) {
		mid = (l + r) / 2; // считываем срединный индекс отрезка [l,r]

		if (dynamicPath.getPointForIndex(mid).arrivalTime > time_solve) r = mid; // проверяем, какую часть нужно отбросить
		else l = mid ;
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


Copter::Copter(string name, double x, double y, double z,
	double speedX, double speedY, double speedZ, 
	double maxAcceleration,
	double inertialXZ, double inertialY,double k_x, double k_v)
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
	this->maxAcceleration = maxAcceleration;
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

void Copter::computeWishData(double time_solve)
{
	int l = 0; 
	int r = dynamicPath.getPath().size() - 1; 
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





