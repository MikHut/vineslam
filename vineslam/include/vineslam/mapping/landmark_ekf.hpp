#pragma once

// Object members
#include <vineslam/params.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Stat.hpp>
#include <vineslam/math/Const.hpp>

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
  KF() = default;
  // Class constructor
  // - Receives the initial state and the parameters
  // - initializes the covariance matrix
  void init3D(const Parameters& params, const VectorXf& X0, const VectorXf& g, const VectorXf& z);
  void init2D(const Parameters& params, const VectorXf& X0, const VectorXf& g, const VectorXf& z);
  // Function that processes all the Kalman Filter routines
  void process3D(const VectorXf& s, const VectorXf& g, const VectorXf& z);
  void process2D(const VectorXf& s, const VectorXf& g, const VectorXf& z);
  // Function that outputs the current state of the Kalman Filter
  Point getState3D() const;
  Point getState2D() const;
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

  // Number of landmark observations
  int n_obsvs_;

  // Function that implements the prediction step of the Kalman Filter
  void predict();
  // Function that implements the update step of the Kalman Filter
  void correct3D(const VectorXf& s, const VectorXf& z);
  void correct2D(const VectorXf& s, const VectorXf& z);
  // Function that calculates the current observations covariance matrix
  void computeR3D(const VectorXf& g, const VectorXf& z);
  void computeR2D(const VectorXf& g, const VectorXf& z);
};

}  // namespace vineslam
