#pragma once

class check {

public:
  // �������� ���������
  static double EPS;

  // ����� �������� �� ��������
	static bool EQ(double value_1, double value_2);

	// ����� �� ��������� �� ��������
	static bool NE(double value_1, double value_2);

	// ����� �������(>) �������� ��� ������, ��� value_1 > value_2
	static bool GT(double value_1, double value_2);

	// ����� �� �������(>=) �������� ��� ������, ��� value_1 > value_2
	static bool GE(double value_1, double value_2);

	// ����� �������(<) �������� ��� ������, ��� value_1 > value_2
	static bool LT(double value_1, double value_2);

	// ����� �� �������(<=) �������� ��� ������, ��� value_1 > value_2
	static bool LE(double value_1, double value_2);

};