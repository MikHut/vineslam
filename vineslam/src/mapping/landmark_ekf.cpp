#include "../../include/vineslam/mapping/landmark_ekf.hpp"

namespace vineslam
{
KF::KF(const Parameters& params, const VectorXf& X0, const VectorXf& g, const VectorXf& z) : X0_(X0), X_(X0)
{
  n_obsvs_ = 1;  // initialization of the number of landmark observations

  // Initialize the process covariance P
  computeR(g, z);
  P_ = R_;
}

void KF::process(const VectorXf& s, const VectorXf& g, const VectorXf& z)
{
  computeR(g, z);
  predict();
  correct(s, z);

  n_obsvs_++;  // increment the number of observations of the landmark
}

void KF::computeR(const VectorXf& g, const VectorXf& z)
{
  //  // Compute the covariance observations matrix based on
  //  // - more noise in depth to far observed landmarks
  //  // - less noise in detection for near observed landmarks
  //  // - the standard deviation of the particle filter pose
  //  //   estimation in both x and y components
  //  R_ = MatrixXf(2, 2);
  //  R_(0, 0) = dispError(z[0]) + g[0];
  //  R_(1, 1) = 1. / (z[0] * z[0]) + g[1];
  //  R_(0, 1) = 0.;
  //  R_(1, 0) = 0.;
  //
  //  // Rotate covariance matrix using the bearing angle
  //  // codified in a rotation matrix
  //  Eigen::MatrixXf Rot(2, 2);
  //  Rot << std::cos(z[1]), -std::sin(z[1]), std::sin(z[1]), std::cos(z[1]);
  //
  //  R_ = Rot * R_ * Rot.transpose();
  R_ = MatrixXf::Identity(3, 3);
  R_(0, 0) = g[0] / n_obsvs_;
  R_(1, 1) = g[1] / n_obsvs_;
  R_(2, 2) = g[2] / n_obsvs_;
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
  float d = std::sqrt(std::pow(X_[0] - s[0], 2) + std::pow(X_[1] - s[1], 2) + std::pow(X_[2] - s[2], 2));
  float yaw = std::atan2(X_[1] - s[1], X_[0] - s[0]) - s[5];
  float pitch = std::atan2(X_[2] - s[2], std::sqrt(std::pow(X_[0] - s[0], 2) + std::pow(X_[1] - s[1], 2))) - s[4];

  VectorXf z_(3, 1);
  z_ << d, Const::normalizeAngle(yaw), Const::normalizeAngle(pitch);

  // Compute the Jacobian of the non linear observation vector
  MatrixXf G(3, 3);
  G(0, 0) = +(X_[0] - s[0]) / d;
  G(0, 1) = +(X_[1] - s[1]) / d;
  G(0, 2) = +(X_[2] - s[2]) / d;
  G(1, 0) = -(X_[1] - s[1]) / std::pow(d, 2);
  G(1, 1) = +(X_[0] - s[0]) / std::pow(d, 2);
  G(1, 2) = 0;
  G(2, 0) = -((X_[2] - s[2]) * (X_[0] - s[0])) /
            (std::pow(d, 2) * std::sqrt(std::pow(X_[0] - s[0], 2) + std::pow(X_[1] - s[1], 2)));
  G(2, 1) = -((X_[2] - s[2]) * (X_[1] - s[1])) /
            (std::pow(d, 2) * std::sqrt(std::pow(X_[0] - s[0], 2) + std::pow(X_[1] - s[1], 2)));
  G(2, 2) = +(std::sqrt(std::pow(X_[0] - s[0], 2) + std::pow(X_[1] - s[1], 2)) / std::pow(d, 2));

  // Compute the Kalman gain
  K_ = P_ * G.transpose() * (G * P_ * G.transpose() + R_).inverse();

  // Compute the difference between the observation vector and the
  // output of the observation model
  VectorXf z_diff = z - z_;
  z_diff[1] = Const::normalizeAngle(z_diff[1]);
  z_diff[2] = Const::normalizeAngle(z_diff[2]);

  // Update the state vector and the process covariance matrix
  Eigen::Vector3f tmp;
  X_ = X_ + K_ * z_diff;
  P_ = (MatrixXf::Identity(3, 3) - K_ * G) * P_;
}

Point KF::getState() const
{
  return Point(X_[0], X_[1], X_[2]);
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
