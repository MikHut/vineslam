#pragma once

#include "const.hpp"
#include "tf.hpp"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace vineslam
{

struct point {
  // Default constructor
  point()
  {
    x = 0.;
    y = 0.;
    z = 0.;
  }

  // Construct with 3D values
  point(const float& x_, const float& y_, const float& z_)
  {
    x = x_;
    y = y_;
    z = z_;
  }

  // Construct with 2D values
  point(const float& x_, const float& y_)
  {
    x = x_;
    y = y_;
    z = 0.;
  }

  // Construct with another point
  point(const point& other)
  {
    x = other.x;
    y = other.y;
    z = other.z;
  }

  // Assignment operator
  point operator=(const point& other)
  {
    x = other.x;
    y = other.y;
    z = other.z;

    return *this;
  }

  // Addition operator
  point operator+(const point& other) const
  {
    point result(*this);
    result.x += other.x;
    result.y += other.y;
    result.z += other.z;

    return result;
  }

  // Subtraction operator
  point operator-(const point& other) const
  {
    point result(*this);
    result.x -= other.x;
    result.y -= other.y;
    result.z -= other.z;

    return result;
  }

  // Division operator
  point operator/(const float& scalar) const
  {
    point result(*this);
    result.x /= scalar;
    result.y /= scalar;
    result.z /= scalar;

    return result;
  }

  // Multiplication by homogeneous transformation
  point operator*(const TF& tf) const
  {
    point res;
    res.x = x * tf.R[0] + y * tf.R[1] + z * tf.R[2] + tf.t[0];
    res.y = x * tf.R[3] + y * tf.R[4] + z * tf.R[5] + tf.t[1];
    res.z = x * tf.R[6] + y * tf.R[7] + z * tf.R[8] + tf.t[2];

    return res;
  }

  // Comparison operators
  bool operator==(const point& other) const
  {
    return (x == other.x && y == other.y && z == other.z);
  }
  bool operator!=(const point& other) const
  {
    return (x != other.x || y != other.y || z != other.z);
  }

  // 3D euclidean distance
  float distance(const point& other) const
  {
    float dist_x = x - other.x;
    float dist_y = y - other.y;
    float dist_z = z - other.z;
    return std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2) +
                     std::pow(dist_z, 2));
  }

  // 2D euclidean distance
  float distanceXY(const point& other) const
  {
    float dist_x = x - other.x;
    float dist_y = y - other.y;
    return std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2));
  }

  // 3D point norm
  float norm3D()
  {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
  }

  // Convert point to Eigen 3D vector
  Eigen::VectorXf toEig2D() const
  {
    Eigen::VectorXf vec(2, 1);
    vec << x, y;
    return vec;
  }

  // Convert point to Eigen 3D vector
  Eigen::VectorXf toEig3D() const
  {
    Eigen::VectorXf vec(3, 1);
    vec << x, y, z;
    return vec;
  }

  // Cartesian coordinates
  float x;
  float y;
  float z;
};

// stdout operator
static std::ostream& operator<<(std::ostream& out, point const& p)
{
  return out << '(' << p.x << ' ' << p.y << ' ' << p.z << ")\n";
}

} // namespace vineslam
