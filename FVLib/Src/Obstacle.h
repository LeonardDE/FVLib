#pragma once
#include "Vector3.h"

enum class RestrictedAreaType
{
	EndlessVerticalCylinder,
	VerticalCylinder,
	
};

class RestrictedArea
{
public:
	RestrictedArea(const Vector3& center, double radius) : center(center), radius(radius) {};

protected:
	Vector3 center;
	double radius;
	RestrictedAreaType type = RestrictedAreaType::EndlessVerticalCylinder;
private:
};

class VerticalRestrictedArea : public RestrictedArea
{
public:

};



class Obstacle
{
public:
	
protected:

private:

};