#include <cmath>
using namespace std;

#include "global.h"

double check::EPS = 0.01;

bool check::EQ(double value_1, double value_2) {
	return abs(value_1 - value_2) < check::EPS;
}

bool check::NE(double value_1, double value_2) {
	return abs(value_1 - value_2) >= check::EPS;
}

bool check::LT(double value_1, double value_2) {
	return value_1 - value_2 < check::EPS;
}

bool check::LE(double value_1, double value_2) {
	return value_1 - value_2 <= check::EPS;
}

bool check::GT(double value_1, double value_2) {
	return value_1 - value_2 > check::EPS;
}

bool check::GE(double value_1, double value_2) {
	return value_1 - value_2 >= check::EPS;
}


// The signum function
int sign(double x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}
