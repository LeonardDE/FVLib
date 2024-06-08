#include "global.h"
#include <cmath>
double EPS = 0.01;

bool check::absCheckDouble(double value_1, double value_2)
{
	return abs(value_1 - value_2) < EPS;
}

bool check::absEqCheckDouble(double value_1, double value_2)
{
	return abs(value_1 - value_2) <= EPS;
}

bool check::lCheckDouble(double value_1, double value_2)
{
	return value_1 - value_2 < EPS;
}
bool check::leqCheckDouble(double value_1, double value_2)
{
	return value_1 - value_2 <= EPS;
}

bool check::rCheckDouble(double value_1, double value_2)
{
	return value_1 - value_2 > EPS;
}
bool check::reqCheckDouble(double value_1, double value_2)
{
	return value_1 - value_2 >= EPS;
}