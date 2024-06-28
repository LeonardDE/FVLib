#include <cmath>
using namespace std;

#include "global.h"

double check::EPS = 0.01;

bool check::EQ(double value_1, double value_2) {
	return abs(value_1 - value_2) <= check::EPS;
}

bool check::NE(double value_1, double value_2) {
	return abs(value_1 - value_2) > check::EPS;
}

bool check::LT(double value_1, double value_2) {
	return value_1 + check::EPS < value_2;
}

bool check::LE(double value_1, double value_2) {
	return value_1 <= value_2 + check::EPS;
}

bool check::GT(double value_1, double value_2) {
	return check::LT(value_2, value_1);
}

bool check::GE(double value_1, double value_2) {
	return check::LE(value_2, value_1);
}


// The signum function
int sign(double x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}
