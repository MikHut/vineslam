#pragma once

// Object members
#include "../params.hpp"
#include "../math/Point.hpp"
#include "../math/stat.hpp"
#include "../math/const.hpp"

// ROS, std, Eigen
#include <eigen3/Eigen/Dense>
#include <iostream>

using Eigen::MatrixXf;
using Eigen::VectorXf;

namespace vineslam
{
class KF
{
public:
  KF(){};
  // Class constructor
  // - Receives the initial state and the parameters
  // - initializes the covariance matrix
  KF(const Parameters& params, const VectorXf& X0, const VectorXf& s, const VectorXf& g, const VectorXf& z);
  // Function that processes all the Kalman Filter routines
  void process(const VectorXf& s, const VectorXf& g, const VectorXf& z);
  // Function that outputs the current state of the Kalman Filter
  Point getState() const;
  // Function that outputs the current standard deviation of the
  // Kalman Filter
  Gaussian<Point, Point> getStdev() const;

  MatrixXf P_;

private:
  // State vector and KF matrices
  VectorXf X0_;
  VectorXf X_;
  MatrixXf K_;
  MatrixXf R_;

  // Input parameters
  float baseline_;
  float delta_d_;
  float fx_;

  // Function that implements the prediction step of the Kalman Filter
  void predict();
  // Function that implements the update step of the Kalman Filter
  void correct(const VectorXf& s, const VectorXf& z);
  // Function that calculates the current observations covariance matrix
  void computeR(const VectorXf& s, const VectorXf& g, const VectorXf& z);

  // Calculates the disparity error using the disparity noise model
  float dispError(const float& depth) const
  {
    return std::pow(depth, 2) / (baseline_ * fx_) * delta_d_;
  }
};

};  // namespace vineslam