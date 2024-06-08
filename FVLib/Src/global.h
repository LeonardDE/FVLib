#pragma once
extern double EPS;

static class check
{
public:
	// Метод строгой(<) проверки  разности по модулю
	static bool absCheckDouble(double value_1, double value_2);

	// Метод не строгой(<=) проверки разности по модулю
	static bool absEqCheckDouble(double value_1, double value_2);

	// Метод строгой(<) проверки без модуля, где value_1 > value_2
	static bool rCheckDouble(double value_1, double value_2);

	// Метод не строгой(<=) проверки без модуля, где value_1 > value_2
	static bool reqCheckDouble(double value_1, double value_2);

	// Метод строгой(>) проверки без модуля, где value_1 > value_2
	static bool lCheckDouble(double value_1, double value_2);

	// Метод не строгой(>=) проверки без модуля, где value_1 > value_2
	static bool leqCheckDouble(double value_1, double value_2);

};