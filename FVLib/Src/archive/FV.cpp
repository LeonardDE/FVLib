

//MaterialPoint::MaterialPoint(string name,
//	const double& x, const double& y, const double& z,
//	const double& speedX, const double& speedY, const double& speedZ,
//	const double& maxAcceleration)
//{
//	this->name = name;
//	this->type = 0;
//	time = 0;
//	curPosition = new vector<double>{ x,y,z };
//	newPosition = new vector<double>{ x,y,z };
//	curVelocity = new vector<double>{ speedX,speedY,speedZ };
//	newVelocity = new vector<double>{ speedX,speedY,speedZ };
//	this->maxAcceleration = maxAcceleration;
//}

//void MaterialPoint::setPath(vector<pair<double, vector<double>>>& path)
//{
//	this->path = &path;
//}

//void MaterialPoint::next
//(
//	double h, double end_time, map<string, void*>& dict
//)
//{
//	vector<double>& u = *(vector<double> *)(dict["acceleration"]);
//	double deltatime = end_time - time;
//	while (deltatime > 0) {
//		for (int i = 0; i < u.size(); i++) {
//			double x = (*curPosition)[i];
//			double v = (*curVelocity)[i];
//
//			(*newPosition)[i] = x + h * v;
//			(*newVelocity)[i] = v + h * u[i];
//			if (deltatime < h)
//			{
//				h = deltatime;
//			}
//			else {
//				deltatime -= h;
//			}
//		}
//	}
//	time = end_time;
//	swap(curPosition, newPosition);
//	swap(curVelocity, newVelocity);
//	/*cout << "AFTER INTEGRATION" << endl;
//	cout << (*curPosition)[0] << ' ' << (*curPosition)[1] << ' ' << (*curPosition)[2] << endl;
//	cout << (*curVelocity)[0] << ' ' << (*curVelocity)[1] << ' ' << (*curVelocity)[2] << endl;*/
//}
//// ѕринимает текущие скорость, врем€, положение и две будущие точки.
//// ¬ыдает два набора: врем€ и управл€ющие параметры. ќдно соответсвует началу маневра, второе к концу 
//// map<string,void*>
//// compute_turn
//map<string,void*> MaterialPoint::turn_compute(const vector<double>& position, const vector<double>& velocity, const double& time,
//	const vector<double>& future_point_1, const vector<double>& future_point_2)
//{
//	vector<double> AC = sub_vec(future_point_1, position);
//	vector<double> CB = sub_vec(future_point_2, future_point_1);
//	double l_AC = norma(AC);
//	double l_CB = norma(CB);
//	double l_velocity = norma(velocity);
//	double cos_phi = scalar_mul_vec(AC, CB) / (l_AC * l_CB);
//	/// Ќаходим вектор ускорени€ дл€ начала поворота
//	vector<double> n = findNormalVector(position,future_point_1,future_point_2,velocity,position);
//	bool isCounterClockwise = orientation(AC, CB, n);
//	vector<double> a_n_1 = isCounterClockwise ? n : digit_mul_vec(n,-1); 
//	///
//	double r = l_velocity * l_velocity / norma(a_n_1);
//	double l = r * sqrt((1 + cos_phi) / (1 - cos_phi));
//
//	double time_1 = (l_AC - l) / l_velocity;
//	double time_2 = acos(cos_phi) * r / l_velocity;
//
//
//	map<string, void*> comandData;
//
//	comandData["turn_time_1"] = new double{ time_1 };
//	comandData["turn_acceleration_1"] = new vector<double>(a_n_1);
//	comandData["turn_time_2"] = new double{ time_2 };
//	comandData["turn_acceleration_2"] = new vector<double>{0,0,0};
//	return comandData;
//	// ¬ычисление точек
//	/*vector<double> start_point = vector<double>{ future_point_1[0] + (-AC[0] * l / l_AC) ,
//		future_point_1[1] + (-AC[1] * l / l_AC) , future_point_1[2] + (-AC[2] * l / l_AC) };
//	vector<double> end_point = vector<double>{ future_point_1[0] + CB[0] / l_CB * l,
//												future_point_1[1] + CB[1] / l_CB * l,
//												future_point_1[2] + CB[2] / l_CB * l };*/
//}


//MaterialPoint::~MaterialPoint()
//{
//	delete curPosition;
//	delete curVelocity;
//	delete newPosition;
//	delete newVelocity;
//}
//
//Plane MaterialPoint::getPlane()
//{
//	Plane plane = Plane();
//	plane.x = (*curPosition)[0];
//	plane.y = (*curPosition)[1];
//	plane.z = (*curPosition)[2];
//	plane.speedX = (*curVelocity)[0];
//	plane.speedY = (*curVelocity)[1];
//	plane.speedZ = (*curVelocity)[2];
//	plane.speed = sqrt(plane.speedX * plane.speedX +
//		plane.speedY * plane.speedY +
//		plane.speedZ * plane.speedZ);
//	plane.name = name;
//	plane.type = type;
//	return plane;
//}
//
//string MaterialPoint::getName()
//{
//	return name;
//}
//
//int32_t MaterialPoint::getType() 
//{
//	return type;
//}
//
//Copter::Copter(string name,
//	const double& x, const double& y, const double& z,
//	const double& speedX, const double& speedY, const double& speedZ,
//	const double& inertialXZ, const double& inertialY)
//{
//	this->name = name;
//	this->type = 1;
//	time = 0;
//
//	this->inertialXZ = inertialXZ;
//	this->inertialY = inertialY;
//
//	curPosition = new vector<double>{ x,y,z };
//	newPosition = new vector<double>{ x,y,z };
//	curVelocity = new vector<double>{ speedX,speedY,speedZ };
//	newVelocity = new vector<double>{ speedX,speedY,speedZ };
//}

//Copter::~Copter()
//{
//	delete curPosition;
//	delete curVelocity;
//	delete newPosition;
//	delete newVelocity;
//}
//
//void Copter::next(double h, double end_time, map<string, void*>& dict)
//{
//	vector<double>& comandVelocity = *(vector<double>*)(dict["comandVelocity"]);
//	while (time < end_time) {
//
//		if (time + h > end_time)
//		{
//			h = end_time - time;
//		}
//		time += h;
//
//		double x = (*curPosition)[0];
//		double v = (*curVelocity)[0];
//
//		(*newPosition)[0] = x + h * v;
//		(*newVelocity)[0] = v + h * (comandVelocity[0] - v) / inertialXZ;
//
//		x = (*curPosition)[1];
//		v = (*curVelocity)[1];
//
//		(*newPosition)[1] = x + h * v;
//		(*newVelocity)[1] = v + h * (comandVelocity[1] - v) / inertialY;
//
//		x = (*curPosition)[2];
//		v = (*curVelocity)[2];
//
//		(*newPosition)[2] = x + h * v;
//		(*newVelocity)[2] = v + h * (comandVelocity[2] - v) / inertialXZ;
//
//		swap(curPosition, newPosition);
//		swap(curVelocity, newVelocity);
//	}
//	time = end_time;
//	swap(curPosition, newPosition);
//	swap(curVelocity, newVelocity);
//}
//Plane Copter::getPlane()
//{
//	Plane plane = Plane();
//	plane.x = (*curPosition)[0];
//	plane.y = (*curPosition)[1];
//	plane.z = (*curPosition)[2];
//	plane.speedX = (*curVelocity)[0];
//	plane.speedY = (*curVelocity)[1];
//	plane.speedZ = (*curVelocity)[2];
//	plane.speed = sqrt(plane.speedX * plane.speedX +
//		plane.speedY * plane.speedY +
//		plane.speedZ * plane.speedZ);
//	plane.name = name;
//	plane.type = type;
//	return plane;
//}