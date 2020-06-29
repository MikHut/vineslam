#pragma once

#include "point.hpp"
#include "const.hpp"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace vineslam
{

class vector3D : public point
{
public:
  vector3D() = default;

  // A vector from three components
  vector3D(const float& a, const float& b, const float& c)
  {
    x = a;
    y = b;
    z = c;
  }

  // A vector resulting from two points: v = p2 - p1
  vector3D(point pt2, point pt1)
  {
    x = pt2.x - pt1.x;
    y = pt2.y - pt1.y;
    z = pt2.z - pt1.z;
  }

  // Cross product between two vectors
  vector3D cross(const vector3D& other)
  {
    vector3D res;
    res.x = +(y * other.z - z * other.y);
    res.y = -(x * other.z - z * other.x);
    res.z = +(x * other.y - y * other.x);

    return res;
  }

  // Dot product between two vectors
  float dot(const vector3D& other)
  {
    return (x * other.x + y * other.y + z * other.z);
  }

  // Normalize vector
  void normalize()
  {
    vector3D tmp = *this;
    x            = tmp.x / tmp.norm();
    y            = tmp.y / tmp.norm();
    z            = tmp.z / tmp.norm();
  }

  // Norm of a vector
  float norm() { return (x * x + y * y + z * z); }

private:
};

} // namespace vineslam
