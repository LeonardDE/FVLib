#pragma once
extern double EPS;

class check
{
public:
	// Метод равенсва по точности
	static bool EQ(double value_1, double value_2);

	// Метод не равенства по сточности
	static bool NOTEQ(double value_1, double value_2);

	// Метод строгой(>) проверки без модуля, где value_1 > value_2
	static bool RE(double value_1, double value_2);

	// Метод не строгой(>=) проверки без модуля, где value_1 > value_2
	static bool REQ(double value_1, double value_2);

	// Метод строгой(<) проверки без модуля, где value_1 > value_2
	static bool LE(double value_1, double value_2);

	// Метод не строгой(<=) проверки без модуля, где value_1 > value_2
	static bool LEQ(double value_1, double value_2);

};