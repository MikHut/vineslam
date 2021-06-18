#include "../../include/vineslam/mapping/landmark_ekf.hpp"

namespace vineslam
{
void KF::init3D(const Parameters& params, const VectorXf& X0, const VectorXf& g, const VectorXf& z)
{
  n_obsvs_ = 1;  // initialization of the number of landmark observations

  // Initialize the process covariance P
  computeR3D(g, z);
  P_ = R_;

  // Initialize the state
  X0_ = X0;
  X_ = X0;
}

void KF::init2D(const Parameters& params, const VectorXf& X0, const VectorXf& g, const VectorXf& z)
{
  n_obsvs_ = 1;  // initialization of the number of landmark observations

  // Initialize the process covariance P
  computeR2D(g, z);
  P_ = R_;

  // Initialize the state
  X0_ = X0;
  X_ = X0;
}

void KF::process3D(const VectorXf& s, const VectorXf& g, const VectorXf& z)
{
  computeR3D(g, z);
  predict();
  correct3D(s, z);

  n_obsvs_++;  // increment the number of observations of the landmark
}

void KF::process2D(const VectorXf& s, const VectorXf& g, const VectorXf& z)
{
  computeR2D(g, z);
  predict();
  correct2D(s, z);

  n_obsvs_++;  // increment the number of observations of the landmark
}

void KF::computeR3D(const VectorXf& g, const VectorXf& z)
{
  R_ = MatrixXf::Identity(3, 3);
  R_(0, 0) = g[0] / n_obsvs_;
  R_(1, 1) = g[1] / n_obsvs_;
  R_(2, 2) = g[2] / n_obsvs_;
}

void KF::computeR2D(const VectorXf& g, const VectorXf& z)
{
  R_ = MatrixXf::Identity(2, 2);
  R_(0, 0) = g[0] / n_obsvs_;
  R_(1, 1) = g[1] / n_obsvs_;
}

void KF::predict()
{
  // Compute the state model - static
  // X_t = X_{t-1}
  // P_t = P_{t-1}
}

void KF::correct3D(const VectorXf& s, const VectorXf& z)
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

void KF::correct2D(const VectorXf& s, const VectorXf& z)
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

Point KF::getState3D() const
{
  return Point(X_[0], X_[1], X_[2]);
}

Point KF::getState2D() const
{
  return Point(X_[0], X_[1], 0);
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
