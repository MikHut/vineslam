#pragma once

#include "Const.hpp"
#include "Tf.hpp"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace vineslam
{
struct Point
{
  // Default constructor
  Point()
  {
    x_ = 0.;
    y_ = 0.;
    z_ = 0.;
    intensity_ = 0.;
  }

  // Construct with 3D values
  Point(const float& x, const float& y, const float& z)
  {
    x_ = x;
    y_ = y;
    z_ = z;
    intensity_ = 0.;
  }

  // Construct with 3D values and intensity
  Point(const float& x, const float& y, const float& z, const float& intensity)
  {
    x_ = x;
    y_ = y;
    z_ = z;
    intensity_ = intensity;
  }

  // Construct with 2D values
  Point(const float& x, const float& y)
  {
    x_ = x;
    y_ = y;
    z_ = 0.;
  }

  // Construct with another point
  Point(const Point& other)
  {
    x_ = other.x_;
    y_ = other.y_;
    z_ = other.z_;
    intensity_ = other.intensity_;
  }

  // Assignment operator
  Point& operator=(const Point& other)
  {
    x_ = other.x_;
    y_ = other.y_;
    z_ = other.z_;
    intensity_ = other.intensity_;

    return *this;
  }

  // Addition operator
  Point operator+(const Point& other) const
  {
    Point result(*this);
    result.x_ += other.x_;
    result.y_ += other.y_;
    result.z_ += other.z_;

    return result;
  }

  // Subtraction operator
  Point operator-(const Point& other) const
  {
    Point result(*this);
    result.x_ -= other.x_;
    result.y_ -= other.y_;
    result.z_ -= other.z_;

    return result;
  }

  // Division operator
  Point operator/(const float& scalar) const
  {
    Point result(*this);
    result.x_ /= scalar;
    result.y_ /= scalar;
    result.z_ /= scalar;

    return result;
  }

  // Scalar multiplication operator
  Point operator*(const float& scalar) const
  {
    Point result(*this);
    result.x_ *= scalar;
    result.y_ *= scalar;
    result.z_ *= scalar;

    return result;
  }

  // Multiplication by homogeneous transformation
  Point operator*(const Tf& tf) const
  {
    Point res;
    res.x_ = x_ * tf.R_array_[0] + y_ * tf.R_array_[1] + z_ * tf.R_array_[2] + tf.t_array_[0];
    res.y_ = x_ * tf.R_array_[3] + y_ * tf.R_array_[4] + z_ * tf.R_array_[5] + tf.t_array_[1];
    res.z_ = x_ * tf.R_array_[6] + y_ * tf.R_array_[7] + z_ * tf.R_array_[8] + tf.t_array_[2];

    return res;
  }

  // Comparison operators
  bool operator==(const Point& other) const
  {
    return (x_ == other.x_ && y_ == other.y_ && z_ == other.z_);
  }
  bool operator!=(const Point& other) const
  {
    return (x_ != other.x_ || y_ != other.y_ || z_ != other.z_);
  }

  // 3D euclidean distance
  float distance(const Point& other) const
  {
    float dist_x = x_ - other.x_;
    float dist_y = y_ - other.y_;
    float dist_z = z_ - other.z_;
    return std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2) + std::pow(dist_z, 2));
  }

  // Squared 3D euclidean distance
  float sqDistance(const Point& other) const
  {
    float dist_x = x_ - other.x_;
    float dist_y = y_ - other.y_;
    float dist_z = z_ - other.z_;
    return std::pow(dist_x, 2) + std::pow(dist_y, 2) + std::pow(dist_z, 2);
  }

  // 2D euclidean distance
  float distanceXY(const Point& other) const
  {
    float dist_x = x_ - other.x_;
    float dist_y = y_ - other.y_;
    return std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2));
  }

  // Squared 2D euclidean distance
  float sqDistanceXY(const Point& other) const
  {
    float dist_x = x_ - other.x_;
    float dist_y = y_ - other.y_;
    return std::pow(dist_x, 2) + std::pow(dist_y, 2);
  }

  // 3D point norm
  float norm3D() const
  {
    return std::sqrt(std::pow(x_, 2) + std::pow(y_, 2) + std::pow(z_, 2));
  }

  // 2D point norm
  float norm2D() const
  {
    return std::sqrt(std::pow(x_, 2) + std::pow(y_, 2));
  }

  // Convert point to Eigen 3D vector
  Eigen::VectorXf toEig2D() const
  {
    Eigen::VectorXf vec(2, 1);
    vec << x_, y_;
    return vec;
  }

  // Convert point to Eigen 3D vector
  Eigen::VectorXf toEig3D() const
  {
    Eigen::VectorXf vec(3, 1);
    vec << x_, y_, z_;
    return vec;
  }

  // Cartesian coordinates
  float x_;
  float y_;
  float z_;

  float intensity_; // used for feature extraction purposes
};

}  // namespace vineslam
