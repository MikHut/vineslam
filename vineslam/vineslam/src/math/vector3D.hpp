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
  vector3D(const point& pt2, const point& pt1)
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

  // Rotation matrix from that aligns this vector with another
  // Source: https://math.stackexchange.com/a/897677
  std::array<float, 9> rotation(vector3D v)
  {
    vector3D ucv  = this->cross(v);
    vector3D vcu  = v.cross(*this);
    float    dval = dot(v);

    Eigen::Matrix3f GG;
    GG(0, 0) = dval;
    GG(0, 1) = -ucv.norm();
    GG(0, 2) = 0;
    GG(1, 0) = ucv.norm();
    GG(1, 1) = dval;
    GG(1, 2) = 0;
    GG(2, 0) = 0;
    GG(2, 1) = 0;
    GG(2, 2) = 1;

    vector3D C(v.x - x * dval, v.y - y * dval, v.z - z * dval);
    float    C_norm = C.norm();
    C.x /= C_norm;
    C.y /= C_norm;
    C.z /= C_norm;

    Eigen::Matrix3f FF;
    FF(0, 0) = x;
    FF(0, 1) = C.x;
    FF(0, 2) = vcu.x;
    FF(1, 0) = y;
    FF(1, 1) = C.y;
    FF(1, 2) = vcu.y;
    FF(2, 0) = z;
    FF(2, 1) = C.z;
    FF(2, 2) = vcu.z;

    Eigen::Matrix3f UU = FF * GG * FF.inverse();

    std::array<float, 9> S{};
    S[0] = UU(0, 0);
    S[1] = UU(0, 1);
    S[2] = UU(0, 2);
    S[3] = UU(1, 0);
    S[4] = UU(1, 1);
    S[5] = UU(1, 2);
    S[6] = UU(2, 0);
    S[7] = UU(2, 1);
    S[8] = UU(2, 2);

    return S;
  }

  // Norm of a vector
  float norm() { return std::sqrt(x * x + y * y + z * z); }

private:
};

} // namespace vineslam
