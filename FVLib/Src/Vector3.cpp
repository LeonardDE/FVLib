#include "Vector3.h"

Vector3::Vector3()
{
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}

Vector3& Vector3::Add(const Vector3& vec)
{
	this->x += vec.x;
	this->y += vec.y;
	this->z += vec.z;
	return *this;
}

Vector3& Vector3::Sub(const Vector3& vec)
{
	this->x -= vec.x;
	this->y -= vec.y;
	this->z -= vec.z;
	return *this;
}

Vector3& Vector3::Mul(const Vector3& vec)
{
	this->x *= vec.x;
	this->y *= vec.y;
	this->z *= vec.z;
	return *this;
}

Vector3& Vector3::Div(const Vector3& vec)
{
	this->x /= vec.x;
	this->y /= vec.y;
	this->z /= vec.z;
	return *this;
}

Vector3& operator+(Vector3& v1, const Vector3& v2) 
{
	return v1.Add(v2);
}
Vector3& operator-(Vector3& v1, const Vector3& v2) 
{
	return v1.Sub(v2);
}
Vector3& operator*(Vector3& v1, const Vector3& v2)
{
	return v1.Mul(v2);
}
Vector3& operator/(Vector3& v1, const Vector3& v2) 
{
	return v1.Div(v2);
}

Vector3& Vector3::operator+=(const Vector3& v2) 
{
	return this->Add(v2);
}
Vector3& Vector3::operator-=(const Vector3& v2)
{
	return this->Sub(v2);
}
Vector3& Vector3::operator*=(const Vector3& v2) 
{
	return this->Mul(v2);
}
Vector3& Vector3::operator/=(const Vector3& v2) 
{
	return this->Div(v2);
}

Vector3& Vector3::Zero()
{
	this->x = 0;
	this->y = 0;
	return *this;
}