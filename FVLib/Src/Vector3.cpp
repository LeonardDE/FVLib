#include <cmath>
using namespace std;

#include "global.h"
#include "Vector3.h"

Vector3::Vector3()
{
  x = 0.0;
  y = 0.0;
  z = 0.0;
}

Vector3::Vector3(const double x, const double y, const double z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

Vector3 &Vector3::Add(const Vector3 &vec)
{
  this->x += vec.x;
  this->y += vec.y;
  this->z += vec.z;
  return *this;
}

Vector3 &Vector3::Sub(const Vector3 &vec)
{
  this->x -= vec.x;
  this->y -= vec.y;
  this->z -= vec.z;
  return *this;
}

Vector3 &Vector3::Mul(const Vector3 &vec)
{
  this->x *= vec.x;
  this->y *= vec.y;
  this->z *= vec.z;
  return *this;
}

Vector3 &Vector3::Div(const Vector3 &vec)
{
  this->x /= vec.x;
  this->y /= vec.y;
  this->z /= vec.z;
  return *this;
}

Vector3 operator+(const Vector3 &v1, const Vector3 &v2)
{
  Vector3 result = v1;

  return result.Add(v2);
}
Vector3 operator-(const Vector3 &v1, const Vector3 &v2)
{
  Vector3 result = v1;

  return result.Sub(v2);
}
Vector3 operator*(const Vector3 &v1, const Vector3 &v2)
{
  Vector3 result = v1;

  return result.Mul(v2);
}
Vector3 operator/(const Vector3 &v1, const Vector3 &v2)
{
  Vector3 result = v1;
  return result.Div(v2);
}

Vector3 operator*(const double &n, const Vector3 &v1)
{
  return v1 * n;
}

Vector3 operator*(const Vector3 &v1, const double &n)
{
  Vector3 result = v1;

  result.x *= n;
  result.y *= n;
  result.z *= n;
  return result;
}

Vector3 operator/(const Vector3 &v1, const double &n)
{
  Vector3 result = v1;
  result.x /= n;
  result.y /= n;
  result.z /= n;
  return result;
}

Vector3 &Vector3::operator+=(const Vector3 &v2)
{
  return this->Add(v2);
}
Vector3 &Vector3::operator-=(const Vector3 &v2)
{
  return this->Sub(v2);
}
Vector3 &Vector3::operator*=(const Vector3 &v2)
{
  return this->Mul(v2);
}
Vector3 &Vector3::operator/=(const Vector3 &v2)
{
  return this->Div(v2);
}

double &Vector3::operator[](int i)
{
  if (i > 2)
  {
    std::cout << "Array index out of bound, exiting";
    exit(0);
  }
  return (i == 0 ? x : (i == 1 ? y : (i == 2 ? z : x)));
}
const double &Vector3::operator[](int i) const
{
  if (i > 2)
  {
    std::cout << "Array index out of bound, exiting";
    exit(0);
  }
  return (i == 0 ? x : (i == 1 ? y : (i == 2 ? z : x)));
}

Vector3 &Vector3::Zero()
{
  this->x = 0;
  this->y = 0;
  this->z = 0;
  return *this;
}

double Vector3::norm() const
{
  return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}

void Vector3::rotate(const Vector3 &axis, const Vector3 &axisPoint, const double &angle)
{
  double c = cos(angle);
  double s = sin(angle);

  double ux = axis.x;
  double uy = axis.y;
  double uz = axis.z;

  double newX = (c + (1 - c) * ux * ux) * (this->x - axisPoint.x) +
                ((1 - c) * ux * uy - s * uz) * (this->y - axisPoint.y) +
                ((1 - c) * ux * uz + s * uy) * (this->z - axisPoint.z) + axisPoint.x;

  double newY = ((1 - c) * ux * uy + s * uz) * (this->x - axisPoint.x) +
                (c + (1 - c) * uy * uy) * (this->y - axisPoint.y) +
                ((1 - c) * uy * uz - s * ux) * (this->z - axisPoint.z) + axisPoint.y;

  double newZ = ((1 - c) * ux * uz - s * uy) * (this->x - axisPoint.x) +
                ((1 - c) * uy * uz + s * ux) * (this->y - axisPoint.y) +
                (c + (1 - c) * uz * uz) * (this->z - axisPoint.z) + axisPoint.z;
  this->x = newX;
  this->y = newY;
  this->z = newZ;
}

Vector3 Vector3::getRotatedVector(const Vector3 &axis, const Vector3 &axisPoint, const double &angle)
{
  double c = cos(angle);
  double s = sin(angle);

  double ux = axis.x;
  double uy = axis.y;
  double uz = axis.z;

  double newX = (c + (1 - c) * ux * ux) * (this->x - axisPoint.x) +
                ((1 - c) * ux * uy - s * uz) * (this->y - axisPoint.y) +
                ((1 - c) * ux * uz + s * uy) * (this->z - axisPoint.z) + axisPoint.x;

  double newY = ((1 - c) * ux * uy + s * uz) * (this->x - axisPoint.x) +
                (c + (1 - c) * uy * uy) * (this->y - axisPoint.y) +
                ((1 - c) * uy * uz - s * ux) * (this->z - axisPoint.z) + axisPoint.y;

  double newZ = ((1 - c) * ux * uz - s * uy) * (this->x - axisPoint.x) +
                ((1 - c) * uy * uz + s * ux) * (this->y - axisPoint.y) +
                (c + (1 - c) * uz * uz) * (this->z - axisPoint.z) + axisPoint.z;
  return Vector3(newX, newY, newZ);
}

Vector3 Vector3::getNormVector()
{
  return *this / this->norm();
}

double scalarProduct(const Vector3 &point_1, const Vector3 &point_2)
{
  return point_1[0] * point_2[0] + point_1[1] * point_2[1] + point_1[2] * point_2[2];
}

double getCosBetweenVectors(const Vector3 &v1, const Vector3 &v2)
{
  return scalarProduct(v1, v2) / (v1.norm() * v2.norm());
}

// double angleBetween(const Vector3& v1, const Vector3& v2)
//{
//	return atan2(getSinBetweenVectors(v1, v2), getCosBetweenVectors(v1, v2));
// }

Vector3 crossProduct(const Vector3 &v1, const Vector3 &v2)
{
  return Vector3(v1[1] * v2[2] - v1[2] * v2[1],
                 v1[2] * v2[0] - v1[0] * v2[2],
                 v1[0] * v2[1] - v1[1] * v2[0]);
}

bool isRightThree(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3)
{
  Vector3 result = crossProduct(v1, v2);
  return scalarProduct(result, v3) > 0;
}

bool eq_points(const Vector3 &point_1, const Vector3 &point_2)
{
  return check::EQ(point_1[0], point_2[0]) &&
         check::EQ(point_1[1], point_2[1]) &&
         check::EQ(point_1[2], point_2[2]);
}

Vector3 findNormalVector(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 point, Vector3 direction)
{
  Vector3 vector1 = Vector3{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
  Vector3 vector2 = Vector3{p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};
  Vector3 planeNormal = crossProduct(vector1, vector2);

  // Проверка, что плоскость задана корректно
  if (planeNormal.x == 0 && planeNormal.y == 0 && planeNormal.z == 0)
  {
    std::cerr << "Некорректное задание плоскости\n";
    return {0, 0, 0};
  }

  Vector3 normalVector = crossProduct(direction, planeNormal);
  normalVector = normalVector / normalVector.norm();

  return normalVector;
}
