#pragma once

// Object members
#include <math/point.hpp>
#include <math/stat.hpp>

// ROS, std, Eigen
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <yaml-cpp/yaml.h>

#define PI 3.14159265359

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
  KF(const VectorXf&    X0,
     const VectorXf&    s,
     const VectorXf&    g,
     const VectorXf&    z,
     const std::string& config_path);
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

  // Auxiliar function that normalizes an angle in the [-pi,pi] range
  float normalizeAngle(const float& angle)
  {
    return (std::fmod(angle + PI, 2 * PI) - PI);
  }

  // Calculates the disparity error using the disparity noise model
  float dispError(const float& depth)
  {
    return pow(depth, 2) / (baseline * fx) * delta_d;
  }
};

}; // namespace vineslam
