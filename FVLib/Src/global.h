#pragma once
extern double EPS;

static class check
{
public:
	// ����� �������(<) ��������  �������� �� ������
	static bool absCheckDouble(double value_1, double value_2);

	// ����� �� �������(<=) �������� �������� �� ������
	static bool absEqCheckDouble(double value_1, double value_2);

	// ����� �������(<) �������� ��� ������, ��� value_1 > value_2
	static bool rCheckDouble(double value_1, double value_2);

	// ����� �� �������(<=) �������� ��� ������, ��� value_1 > value_2
	static bool reqCheckDouble(double value_1, double value_2);

	// ����� �������(>) �������� ��� ������, ��� value_1 > value_2
	static bool lCheckDouble(double value_1, double value_2);

	// ����� �� �������(>=) �������� ��� ������, ��� value_1 > value_2
	static bool leqCheckDouble(double value_1, double value_2);

};