#include "FV.h"
#include "GlobalSituation.h"
#include "global.h"
#include <cmath>

void FV::setPath(vector<Point> path)
{
	
	this->basePath = path;
	//double turnAcceleration = this->maxAcceleration / 2;

	Path new_path = Path();

	if (path.size() > 2)
	{
		for (int i = 0; i < path.size() - 2; i++)
		{
			Vector3 AB = path[i + 1].position - path[i ].position;
			Vector3 BC = path[i + 2].position - path[i + 1].position;
			double cos_a = getCosBetweenVectors(AB * (-1), BC);
			
			if (abs(cos_a) < 0.99)
			{
				Vector3 vel_1 = AB / (path[i + 1].arrivalTime - path[i].arrivalTime);
				Vector3 vel_2 = BC / (path[i + 2].arrivalTime - path[i + 1].arrivalTime);

				double r = solveTurnRadius(vel_1,vel_2);
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

void FV::checkConflict()
{
	vector<FV*>& fvs = globalSituation->FVs;
	map<string, FVState>& states = globalSituation->aetherInfo.states;
	for (auto& fv : fvs)
	{
		string fv_name = fv->getPlane().name;
		if (fv_name == this->name) continue;

		Plane plane = getPlane();
		Vector3 pos = Vector3(plane.x, plane.y, plane.z);
		
		Vector3 hisPos = Vector3(states[fv_name].plane.x, 
								 states[fv_name].plane.y, 
								 states[fv_name].plane.z);
		Vector3 relPosition = hisPos - pos;

		if (relPosition.norm() > radiusFilter) continue;

		Vector3 vel = Vector3(plane.speedX, plane.speedY, plane.speedZ);

		Vector3 hisVel = Vector3(states[fv_name].plane.speedX, 
								 states[fv_name].plane.speedY, 
								 states[fv_name].plane.speedZ);

		if (scalarProduct(relPosition, hisVel - vel) < 0) continue;

		// Сюда нужна проверка с норированием сетки
		
		// Нормируем сетки
		vector<Point> my_plan;
		vector<Point> his_plan;
		if (states[this->name].shortPlan.empty() || states[fv_name].shortPlan.empty()) continue;

		mergeShortPlans(states[this->name].shortPlan, states[fv_name].shortPlan, my_plan, his_plan);
		double left_t_min_xz = INFINITY;
		double right_t_max_xz = -INFINITY;

		double left_t_min_y = INFINITY;
		double right_t_max_y = -INFINITY;

		// Проверка
		for (int i = 0; i < my_plan.size() - 1; i++)
		{
			// Проверка по XZ
			/// Считаем коэффэциенты  для квадратного уравнения
			Vector3 my_vel = (my_plan[i + 1].position - my_plan[i].position) / (my_plan[i + 1].arrivalTime - my_plan[i].arrivalTime);
			Vector3 his_vel = (his_plan[i + 1].position - his_plan[i].position) / (his_plan[i + 1].arrivalTime - his_plan[i].arrivalTime);

			double A = (my_vel.x - his_vel.x)* (my_vel.x - his_vel.x) + (my_vel.z - his_vel.z) * (my_vel.z - his_vel.z);
			
			double B = 2 * (my_plan[i].arrivalTime * A + ((my_plan[i].position.x - his_plan[i].position.x) + (my_vel.x - hisVel.x)) 
														* ((my_plan[i].position.z - his_plan[i].position.z) + (my_vel.z - hisVel.z)));
			
			double C = -2 * my_plan[i].arrivalTime * ((my_plan[i].position.x - his_plan[i].position.x) + (my_vel.x - hisVel.x))
													* ((my_plan[i].position.z - his_plan[i].position.z) + (my_vel.z - hisVel.z))
						+ my_plan[i].arrivalTime * my_plan[i].arrivalTime * A 
						+ (my_plan[i].position.x - his_plan[i].position.x) * (my_plan[i].position.z - his_plan[i].position.z)
						- radiusWarn;
			
			/// Находим корни
			double D = B * B - 4 * A * C;
			if (D >= 0) 
			{
				double t1 = (-B - sqrt(D)) / (2 * A);
				double t2 = (-B + sqrt(D)) / (2 * A);

				left_t_min_xz = min(t1,left_t_min_xz);
				right_t_max_xz = max(t2, right_t_max_xz);
			}
			// Проверка по Y
			double Ay = (my_vel.y - his_vel.y) * (my_vel.y - his_vel.y);

			double By = 2 * (my_vel.y - his_vel.y) * ((my_plan[i].position.y - his_plan[i].position.y) + my_plan[i].arrivalTime * (my_vel.y - his_vel.y));

			double Cy = ((my_plan[i].position.y - his_plan[i].position.y) + my_plan[i].arrivalTime * (my_vel.y - his_vel.y))
							* ((my_plan[i].position.y - his_plan[i].position.y) + my_plan[i].arrivalTime * (my_vel.y - his_vel.y))
						- heightWarn;
			
			/// Находим корни
			double Dy = By * By - 4 * Ay * Cy;
			if (Dy >= 0) {
				double t1_y = (-By - sqrt(Dy)) / (2 * Ay);
				double t2_y = (-By + sqrt(Dy)) / (2 * Ay);

				left_t_min_y = min(t1_y, left_t_min_y);
				right_t_max_y = max(t2_y, right_t_max_y);
			}

		}
		if ((left_t_min_xz - right_t_max_y) * (left_t_min_y - right_t_max_xz) <= 0) return;



		json conflict;
		conflict["name"] = fv->name;
		conflict["tBeg"] = max(left_t_min_xz, left_t_min_y);
		conflict["tEnd"] = min(right_t_max_xz, right_t_max_y);

		conflicts.push_back(conflict);
		cout << conflicts.dump(1) << endl;
		
		// ну и вот тут надо что то делать так как мы понмиаем что у нас тут конфликт
	}
}

 



void mergeShortPlans(const vector<Point>& arr1, const vector<Point>& arr2,
	vector<Point>& res1, vector<Point>& res2)
{
	//res1.clear();
	//res2.clear();

	size_t i = 0;
	size_t j = 0;

	double maxStart = max(arr1[0].arrivalTime, arr2[0].arrivalTime);
	double minEnd = min(arr2[arr2.size() - 1].arrivalTime, arr1[arr1.size() - 1].arrivalTime);

	while (i < arr1.size() && j < arr2.size()) {
		if (arr1[i].arrivalTime + EPS < arr2[j].arrivalTime) {
			if (arr1[i].arrivalTime >= maxStart && arr1[i].arrivalTime <= minEnd)
			{
				Point p;
				p.arrivalTime = arr1[i].arrivalTime;
				p.position = arr2[j-1].position + (arr2[j].position - arr2[j-1].position)
					* (p.arrivalTime - arr2[j - 1].arrivalTime)
					/ (arr2[j].arrivalTime - arr2[j-1].arrivalTime);
				res2.push_back(p);
				res1.push_back(arr1[i]);
			}
			i++;
		}
		else if (arr1[i].arrivalTime - EPS > arr2[j].arrivalTime) {
			if (arr2[j].arrivalTime >= maxStart && arr2[j].arrivalTime <= minEnd)
			{
				Point p;
				p.arrivalTime = arr2[j].arrivalTime;
				p.position = arr1[i-1].position + (arr1[i].position - arr1[i - 1].position)
					* (p.arrivalTime - arr1[i - 1].arrivalTime)
					/ (arr1[i].arrivalTime - arr1[i - 1].arrivalTime);
				res1.push_back(p);
				res2.push_back(arr2[j]);
			}
			j++;
		}
		else
		{
			res1.push_back(arr1[i]);
			res2.push_back(arr2[j]);
			i++;
			j++;
		}
	}
}

void FV::doBroadcast()
{
	if (nextBroadcastInstant <= time)
	{
		nextBroadcastInstant += broadcastStep;
		Plane plane = getPlane();
		globalSituation->aetherInfo.broadcastState(plane);

		nextBroadcastInstant += broadcastStep;
	}


	if (globalSituation->aetherInfo.states[this->name].shortPlan.empty() ||
		globalSituation->aetherInfo.states[this->name].shortPlan[0].arrivalTime - EPS <= time)
	{
		Plane plane = getPlane();
		vector<Point> short_plan;
		for (auto& p : dynamicPath.getPath())
		{

			if (short_plan.size() > 3) break;
			if (p.arrivalTime < time) continue;
			short_plan.push_back(Point(p.position, p.arrivalTime));
		}

		globalSituation->aetherInfo.broadcastPlan(plane.name, time, short_plan);
	}
}
