#pragma once

class check {

public:
  // Точность сравнения
  static double EPS;

  // Метод равенства по точности
	static bool EQ(double value_1, double value_2);

	// Метод не равенства по точности
	static bool NE(double value_1, double value_2);

	// Метод строгой(>) проверки без модуля, где value_1 > value_2
	static bool GT(double value_1, double value_2);

	// Метод нестрогой(>=) проверки без модуля, где value_1 > value_2
	static bool GE(double value_1, double value_2);

	// Метод строгой(<) проверки без модуля, где value_1 > value_2
	static bool LT(double value_1, double value_2);

	// Метод нестрогой(<=) проверки без модуля, где value_1 > value_2
	static bool LE(double value_1, double value_2);

};


// The signum function
int sign(double x);