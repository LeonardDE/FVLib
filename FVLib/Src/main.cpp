#include <iostream>
#include <string>
#include "Modeler.h"
#include "FV.h"
using namespace std;



int main()
{
	string file_name = "test.json";
	Modeler modeler = Modeler(file_name);
	
	modeler.startModeling();

	return 0;
}

//int main()
//{
//	output = fopen("position.dat", "w");
//	velocity_file = fopen("velocity.dat", "w");
//	path_file = fopen("path.dat", "w");
//	path_file_2 = fopen("path2.dat", "w");
//	acceleration_file = fopen("acceleration.dat", "w");
//	wishPos = fopen("wishPos.dat", "w");
//	wishVel = fopen("wishVel.dat", "w");
//
//	// Получать время
//	// Получаем шаг вычисления состояния
//	// Получаем шаг интегрирования для моделей/аппаратов
//	double solve_time = 1;
//	double integral_h = 0.001;
//	double h = 0.001;
//	double timer = 0;
//
//	MaterialPoint mPoint = MaterialPoint("point",0,0,0,0,0,0,1,-1,-1);
//
//	Vector3 p1 = Vector3(  0,0,0 );
//	Vector3 p2 = Vector3(  10,0,0);
//	Vector3 p3 = Vector3(  10,0,-5 );
//	Vector3 p4 = Vector3( 15, 0, -10);
//
//	vector<Point> path =
//	{
//		Point(p1,0),
//		Point{p2,10},
//		Point{p3,20},
//		Point{p4,25},
//	};
//
//	mPoint.setPath(path);
//
//	for (const Point& pathPoint : path//mPoint.getPath()
//		)
//	{
//		fprintf(path_file, "%f %f\n", pathPoint.position.x, pathPoint.position.z);
//	}
//
//	for (PathPoint pathPoint : mPoint.getDynamicPath().getPath())
//	{
//		pathPoint.position.print("POINT and Time");
//		cout << pathPoint.arrivalTime << endl;
//		cout << (int)pathPoint.type << endl;
//		fprintf(path_file_2, "%f %f\n", pathPoint.position.x, pathPoint.position.z);
//	}
//
//	Plane plane;
//	while (timer < 25)
//	{
//		mPoint.next(0.01, timer);
//		plane = mPoint.getPlane();
//		cout << "-------" << timer << "------" << endl;
//		cout << "Position(" << plane.x << ", " << plane.y << ", " << plane.z << ")" << endl;
//		cout << "Velocity(" << plane.speedX << ", " << plane.speedY << ", " << plane.speedZ << ")" << endl;
//		cout << "|Velocity| = " << plane.speed << endl;
//		cout << "-------------" << endl;
//
//		fprintf(output, "%f %f\n", plane.x, plane.z);
//		fprintf(velocity_file, "%f %f %f\n", timer,plane.speedX, plane.speedZ);
//		fprintf(acceleration_file, "%f %f\n",  timer, mPoint.acceleration.norm());
//
//		fprintf(wishPos, "%f %f %f\n", mPoint.wishPosition.x, mPoint.wishPosition.z, mPoint.wishPosition.y);
//		fprintf(wishVel, "%f %f\n", timer,mPoint.wishVelocity.norm());
//
//		timer+=0.1;
//	}
//	fclose(output);
//	fclose(velocity_file);
//	fclose(acceleration_file);
//	fclose(path_file);
//	fclose(wishPos);
//	fclose(wishVel);
//
//	return 0;
//}




