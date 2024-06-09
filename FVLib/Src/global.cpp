#include "global.h"
#include <cmath>
double EPS = 0.01;

bool check::EQ(double value_1, double value_2)
{
	return abs(value_1 - value_2) < EPS;
}

static bool NOTEQ(double value_1, double value_2)
{
	return abs(value_1 - value_2) >= EPS;
}

bool check::LE(double value_1, double value_2)
{
	return value_1 - value_2 < EPS;
}
bool check::LEQ(double value_1, double value_2)
{
	return value_1 - value_2 <= EPS;
}

bool check::RE(double value_1, double value_2)
{
	return value_1 - value_2 > EPS;
}
bool check::REQ(double value_1, double value_2)
{
	return value_1 - value_2 >= EPS;
}