#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#define PI 3.14159265359

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

  // 3D euclidean distance
  float distance(const point& other) const
  {
    float dist_x = x - other.x;
    float dist_y = y - other.y;
    float dist_z = z - other.z;
    return sqrt(pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_z, 2));
  }

  // 2D euclidean distance
  float distanceXY(const point& other) const
  {
    float dist_x = x - other.x;
    float dist_y = y - other.y;
    return sqrt(pow(dist_x, 2) + pow(dist_y, 2));
  }

  // Convert point to Eigen 3D vector
  Eigen::VectorXf toEig2D()
  {
    Eigen::VectorXf vec(2, 1);
    vec << x, y;
    return vec;
  }

  // Convert point to Eigen 3D vector
  Eigen::VectorXf toEig3D()
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
}; // namespace vineslam
