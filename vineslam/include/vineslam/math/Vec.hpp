#pragma once

#include "Point.hpp"
#include "Const.hpp"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace vineslam
{
class Vec : public Point
{
public:
  Vec() = default;

  // A vector from three components
  Vec(const float& a, const float& b, const float& c)
  {
    x_ = a;
    y_ = b;
    z_ = c;
  }

  // A vector resulting from two points: v = p2 - p1
  Vec(const Point& pt2, const Point& pt1)
  {
    x_ = pt2.x_ - pt1.x_;
    y_ = pt2.y_ - pt1.y_;
    z_ = pt2.z_ - pt1.z_;
  }

  // Cross product between two vectors
  Vec cross(const Vec& other)
  {
    Vec res;
    res.x_ = +(y_ * other.z_ - z_ * other.y_);
    res.y_ = -(x_ * other.z_ - z_ * other.x_);
    res.z_ = +(x_ * other.y_ - y_ * other.x_);

    return res;
  }

  // Dot product between two vectors
  float dot(const Vec& other)
  {
    return (x_ * other.x_ + y_ * other.y_ + z_ * other.z_);
  }

  // Normalize vector
  void normalize()
  {
    Vec tmp = *this;
    x_ = tmp.x_ / tmp.norm();
    y_ = tmp.y_ / tmp.norm();
    z_ = tmp.z_ / tmp.norm();
  }

  // Norm of a vector
  float norm()
  {
    return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
  }

  // Multiplication by homogeneous transformation
  Vec operator*(const Tf& tf) const
  {
    Vec res;
    res.x_ = x_ * tf.R_array_[0] + y_ * tf.R_array_[1] + z_ * tf.R_array_[2] + tf.t_array_[0];
    res.y_ = x_ * tf.R_array_[3] + y_ * tf.R_array_[4] + z_ * tf.R_array_[5] + tf.t_array_[1];
    res.z_ = x_ * tf.R_array_[6] + y_ * tf.R_array_[7] + z_ * tf.R_array_[8] + tf.t_array_[2];

    return res;
  }

  // Rotation matrix from that aligns this vector with another
  // Source: https://math.stackexchange.com/a/897677
  std::array<float, 9> rotation(Vec v)
  {
    Vec ucv = this->cross(v);
    Vec vcu = v.cross(*this);
    float dval = dot(v);

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

    Vec C(v.x_ - x_ * dval, v.y_ - y_ * dval, v.z_ - z_ * dval);
    float C_norm = C.norm();
    C.x_ /= C_norm;
    C.y_ /= C_norm;
    C.z_ /= C_norm;

    Eigen::Matrix3f FF;
    FF(0, 0) = x_;
    FF(0, 1) = C.x_;
    FF(0, 2) = vcu.x_;
    FF(1, 0) = y_;
    FF(1, 1) = C.y_;
    FF(1, 2) = vcu.y_;
    FF(2, 0) = z_;
    FF(2, 1) = C.z_;
    FF(2, 2) = vcu.z_;

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


private:
};

}  // namespace vineslam
