#pragma once
#include <iostream>
class Vector3
{
public:
	float x;
	float y;
	float z;
	Vector3();
	Vector3(const float x, const float y, const float z) : x(x), y(y), z(z) {}

	Vector3& Add(const Vector3& vec);
	Vector3& Sub(const Vector3& vec);
	Vector3& Mul(const Vector3& vec);
	Vector3& Div(const Vector3& vec);

	friend Vector3& operator+(Vector3& v1, const Vector3& v2);
	friend Vector3& operator-(Vector3& v1, const Vector3& v2);
	friend Vector3& operator*(Vector3& v1, const Vector3& v2);
	friend Vector3& operator/(Vector3& v1, const Vector3& v2);


	Vector3& operator+=(const Vector3& v2);
	Vector3& operator-=(const Vector3& v2);
	Vector3& operator*=(const Vector3& v2);
	Vector3& operator/=(const Vector3& v2);

	Vector3& Zero();
	friend std::ostream& operator<<(std::ostream& stream, const Vector3& vec);
};