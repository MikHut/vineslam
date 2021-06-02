#include "../../include/vineslam/mapping/landmark_ekf.hpp"

namespace vineslam
{
KF::KF(const Parameters& params, const VectorXf& X0, const VectorXf& g, const VectorXf& z)
  : X0_(X0), X_(X0)
{
  fx_ = params.fx_;
  baseline_ = params.baseline_;
  delta_d_ = 0.1;

  // Initialize the process covariance P
  computeR(g, z);
  P_ = R_;
}

void KF::process(const VectorXf& s, const VectorXf& g, const VectorXf& z)
{
  computeR(g, z);
  predict();
  correct(s, z);
}

void KF::computeR(const VectorXf& g, const VectorXf& z)
{
  // Compute the covariance observations matrix based on
  // - more noise in depth to far observed landmarks
  // - less noise in detection for near observed landmarks
  // - the standard deviation of the particle filter pose
  //   estimation in both x and y components
  R_ = MatrixXf(2, 2);
  R_(0, 0) = dispError(z[0]) + g[0];
  R_(1, 1) = 1. / (z[0] * z[0]) + g[1];
  R_(0, 1) = 0.;
  R_(1, 0) = 0.;

  // Rotate covariance matrix using the bearing angle
  // codified in a rotation matrix
  Eigen::MatrixXf Rot(2, 2);
  Rot << std::cos(z[1]), -std::sin(z[1]), std::sin(z[1]), std::cos(z[1]);

  R_ = Rot * R_ * Rot.transpose();
}

void KF::predict()
{
  // Compute the state model - static
  // X_t = X_{t-1}
  // P_t = P_{t-1}
}

void KF::correct(const VectorXf& s, const VectorXf& z)
{
  // Apply the observation model using the current state vector
  float d = std::sqrt(std::pow(X_[0] - s[0], 2) + std::pow(X_[1] - s[1], 2));
  float phi = std::atan2(X_[1] - s[1], X_[0] - s[0]) - s[2];

  VectorXf z_(2, 1);
  z_ << d, Const::normalizeAngle(phi);

  // Compute the Jacobian of the non linear observation vector
  MatrixXf G(2, 2);
  G << (X_[0] - s[0]) / d, +(X_[1] - s[1]) / d, -(X_[1] - s[1]) / pow(d, 2), (X_[0] - s[0]) / pow(d, 2);

  // Compute the Kalman gain
  K_ = P_ * G.transpose() * (G * P_ * G.transpose() + R_).inverse();

  // Compute the difference between the observation vector and the
  // output of the observation model
  VectorXf z_diff = z - z_;
  z_diff[1] = Const::normalizeAngle(z_diff[1]);

  // Update the state vector and the process covariance matrix
  X_ = X_ + K_ * z_diff;
  P_ = (MatrixXf::Identity(2, 2) - K_ * G) * P_;
}

Point KF::getState() const
{
  return Point(X_[0], X_[1], 0.);
}

Gaussian<Point, Point> KF::getStdev() const
{
  Eigen::EigenSolver<Eigen::Matrix2f> s(P_);
  Eigen::Vector2cf eigvec = s.eigenvectors().col(1);
  Eigen::Vector2cf eigval = s.eigenvalues();
  float angle = std::atan2(eigvec.real()[1], eigvec.real()[0]);

  Point m_mean(X_[0], X_[1], X_[2]);
  Point m_stdev(eigval.real()[0], eigval.real()[1], 0.);
  return Gaussian<Point, Point>(m_mean, m_stdev, angle);
}

}  // namespace vineslam
