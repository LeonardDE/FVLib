#include "FV.h"

MaterialPoint::MaterialPoint(string name,
	const double& x, const double& y, const double& z,
	const double& speedX, const double& speedY, const double& speedZ)
{
	this->name = name;
	this->type = 0;
	time = 0;
	curPosition = new vector<double>{ x,y,z };
	newPosition = new vector<double>{ x,y,z };
	curVelocity = new vector<double>{ speedX,speedY,speedZ };
	newVelocity = new vector<double>{ speedX,speedY,speedZ };
}


void MaterialPoint::next
(
	double h, double end_time, map<string, void*>& dict
	//const std::vector<double>& u, double end_time
)
{
	vector<double>& u = *(vector<double> *)(dict["acceleration"]);
	double deltatime = end_time - time;
	while (deltatime > 0) {
		for (int i = 0; i < u.size(); i++) {
			double x = (*curPosition)[i];
			double v = (*curVelocity)[i];

			(*newPosition)[i] = x + h * v;
			(*newVelocity)[i] = v + h * u[i];
			if (deltatime < h)
			{
				h = deltatime;
			}
			else {
				deltatime -= h;
			}
		}
	}
	time = end_time;
	swap(curPosition, newPosition);
	swap(curVelocity, newVelocity);
	/*cout << "AFTER INTEGRATION" << endl;
	cout << (*curPosition)[0] << ' ' << (*curPosition)[1] << ' ' << (*curPosition)[2] << endl;
	cout << (*curVelocity)[0] << ' ' << (*curVelocity)[1] << ' ' << (*curVelocity)[2] << endl;*/
}

MaterialPoint::~MaterialPoint()
{
	delete[] curPosition;
	delete[] curVelocity;
	delete[] newPosition;
	delete[] newVelocity;
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

Copter::Copter(string name,
	const double& x, const double& y, const double& z,
	const double& speedX, const double& speedY, const double& speedZ,
	const double& wishSpeedX, const double& wishSpeedY, const double& wishSpeedZ)
{
	this->name = name;
	this->type = 1;
	time = 0;

	wishVelocity = new vector<double>{ wishSpeedX,wishSpeedY,wishSpeedZ };

	curPosition = new vector<double>{ x,y,z };
	newPosition = new vector<double>{ x,y,z };
	curVelocity = new vector<double>{ speedX,speedY,speedZ };
	newVelocity = new vector<double>{ speedX,speedY,speedZ };
}

Copter::~Copter()
{
	delete[] curPosition;
	delete[] curVelocity;
	delete[] newPosition;
	delete[] newVelocity;

	delete[] wishVelocity;
}

void Copter::next(double h, double end_time, map<string, void*>& dict)
{
	vector<double>& u = *(vector<double> *)(dict["acceleration"]);
	double& i_x = *(double*)(dict["inertialX"]);
	double& i_y = *(double*)(dict["inertialY"]);
	double deltatime = end_time - time;
	while (deltatime > 0) {
		double x = (*curPosition)[0];
		double v = (*curVelocity)[0];

		(*newPosition)[0] = x + h * v;
		(*newVelocity)[0] = v + h * (pow((*wishVelocity)[0], u[0]) - v) / i_x;

		double x = (*curPosition)[1];
		double v = (*curVelocity)[1];

		(*newPosition)[1] = x + h * v;
		(*newVelocity)[1] = v + h * (pow((*wishVelocity)[1], u[1]) - v) / i_y;

		double x = (*curPosition)[2];
		double v = (*curVelocity)[2];

		(*newPosition)[2] = x + h * v;
		(*newVelocity)[2] = v + h * (pow((*wishVelocity)[2], u[2]) - v) / i_x;


		if (deltatime < h)
		{
			h = deltatime;
		}
		else {
			deltatime -= h;
		}

	}
	time = end_time;
	swap(curPosition, newPosition);
	swap(curVelocity, newVelocity);
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