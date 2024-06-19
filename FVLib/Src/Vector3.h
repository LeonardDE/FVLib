#pragma once

#include <iostream>
using namespace std;

class Vector3 {
public:
  double x;
  double y;
  double z;
  Vector3();
  Vector3(const double x, const double y, const double z);

  Vector3& Add(const Vector3& vec);
  Vector3& Sub(const Vector3& vec);
  Vector3& Mul(const Vector3& vec);
  Vector3& Div(const Vector3& vec);

  friend Vector3 operator+(const Vector3& v1, const Vector3& v2);
  friend Vector3 operator-(const Vector3& v1, const Vector3& v2);
  friend Vector3 operator*(const Vector3& v1, const Vector3& v2);
  friend Vector3 operator/(const Vector3& v1, const Vector3& v2);

  friend Vector3 operator*(const Vector3& v1, const double& n);
  friend Vector3 operator*(const double& n, const Vector3& v1);
  friend Vector3 operator/(const Vector3& v1, const double& n);

  Vector3& operator+=(const Vector3& v2);
  Vector3& operator-=(const Vector3& v2);
  Vector3& operator*=(const Vector3& v2);
  Vector3& operator/=(const Vector3& v2);
  double& operator[](int i);
  const double& operator[](int i) const;

  Vector3& Zero();
  double norm() const;
  void rotate(const Vector3& axis, const Vector3& axisPoint, const double& angle);
  Vector3 getRotatedVector(const Vector3& axis, const Vector3& axisPoint, const double& angle);

  friend double scalarProduct(const Vector3& point_1, const Vector3& point_2);
  friend Vector3 crossProduct(const Vector3& v1, const Vector3& v2);
  friend bool isRightThree(const Vector3& v1, const Vector3& v2, const Vector3& v3);
  friend Vector3 findNormalVector(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 point, Vector3 direction);
  friend double getCosBetweenVectors(const Vector3& v1, const Vector3& v2);
  friend double distance(const Vector3& v1, const Vector3& v2);
  
  Vector3 getNormVector();

  void print(string name = "") {
    if (name != "") cout << name << endl;
    cout << "(" << this->x << ", " << this->y << ", " << this->z << ")" << endl;
  }
};
