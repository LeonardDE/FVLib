#pragma once
extern double EPS;

class check
{
public:
	// ����� �������� �� ��������
	static bool EQ(double value_1, double value_2);

	// ����� �� ��������� �� ���������
	static bool NOTEQ(double value_1, double value_2);

	// ����� �������(>) �������� ��� ������, ��� value_1 > value_2
	static bool RE(double value_1, double value_2);

	// ����� �� �������(>=) �������� ��� ������, ��� value_1 > value_2
	static bool REQ(double value_1, double value_2);

	// ����� �������(<) �������� ��� ������, ��� value_1 > value_2
	static bool LE(double value_1, double value_2);

	// ����� �� �������(<=) �������� ��� ������, ��� value_1 > value_2
	static bool LEQ(double value_1, double value_2);

};