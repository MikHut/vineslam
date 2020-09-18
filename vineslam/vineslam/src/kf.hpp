#pragma once

// Object members
#include <params.hpp>
#include <math/point.hpp>
#include <math/stat.hpp>
#include <math/const.hpp>

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
  KF(const Parameters&  params,
     const VectorXf&    X0,
     const VectorXf&    s,
     const VectorXf&    g,
     const VectorXf&    z);
  // Function that processes all the Kalman Filter routines
  void process(const VectorXf& s, const VectorXf& g, const VectorXf& z);
  // Function that outputs the current state of the Kalman Filter
  point getState() const;
  // Function that outputs the current standard deviation of the
  // Kalman Filter
  Gaussian<point, point> getStdev() const;

  MatrixXf P;

private:
  // State vector and KF matrices
  VectorXf X0;
  VectorXf X;
  MatrixXf K;
  MatrixXf R;

  // Input parameters
  float baseline;
  float delta_d;
  float fx;

  // Function that implements the prediction step of the Kalman Filter
  void predict();
  // Function that implements the update step of the Kalman Filter
  void correct(const VectorXf& s, const VectorXf& z);
  // Function that calculates the current observations covariance matrix
  void computeR(const VectorXf& s, const VectorXf& g, const VectorXf& z);

  // Calculates the disparity error using the disparity noise model
  float dispError(const float& depth) const
  {
    return std::pow(depth, 2) / (baseline * fx) * delta_d;
  }
};

}; // namespace vineslam
