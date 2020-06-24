#include "kf.hpp"

namespace vineslam
{
KF::KF(const VectorXf&    X0,
       const VectorXf&    s,
       const VectorXf&    g,
       const VectorXf&    z,
       const std::string& config_path)
    : X0(X0)
    , X(X0)
{
  // Load input parameters
  YAML::Node config = YAML::LoadFile(config_path);
  fx                = config["camera_info"]["fx"].as<float>();
  baseline          = config["camera_info"]["baseline"].as<float>();
  delta_d           = config["camera_info"]["delta_d"].as<float>();

  // Initialize the process covariance P
  computeR(s, g, z);
  P = R;
}

void KF::process(const VectorXf& s, const VectorXf& g, const VectorXf& z)
{
  computeR(s, g, z);
  predict();
  correct(s, z);
}

void KF::computeR(const VectorXf& s, const VectorXf& g, const VectorXf& z)
{
  // Compute the covariance observations matrix based on
  // - more noise in depth to far observed landmarks
  // - less noise in detection for near observed landmarks
  // - the standard deviation of the particle filter pose
  //   estimation in both x and y components
  R       = MatrixXf(2, 2);
  R(0, 0) = dispError(z[0]) + g[0];
  R(1, 1) = 0.1 / (z[0] * z[0]) + g[1];
  R(0, 1) = 0;
  R(1, 0) = 0;

  // Rotate covariance matrix using the bearing angle
  // codified in a rotation matrix
  Eigen::MatrixXf Rot(2, 2);
  Rot << std::cos(z[1]), -std::sin(z[1]), std::sin(z[1]), std::cos(z[1]);

  R = Rot * R * Rot.transpose();
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
  float d   = std::sqrt(pow(X[0] - s[0], 2) + pow(X[1] - s[1], 2));
  float phi = std::atan2(X[1] - s[1], X[0] - s[0]) - s[2];

  VectorXf z_(2, 1);
  z_ << d, normalizeAngle(phi);

  // Compute the Jacobian of the non linear observation vector
  MatrixXf G(2, 2);
  G << (X[0] - s[0]) / d, +(X[1] - s[1]) / d, -(X[1] - s[1]) / pow(d, 2),
      (X[0] - s[0]) / pow(d, 2);

  // Compute the Kalman gain
  K = P * G.transpose() * (G * P * G.transpose() + R).inverse();

  // Compute the difference between the observation vector and the
  // output of the observation model
  VectorXf z_diff = z - z_;
  z_diff[1]       = normalizeAngle(z_diff[1]);

  // Update the state vector and the process covariance matrix
  X = X + K * z_diff;
  P = (MatrixXf::Identity(2, 2) - K * G) * P;
}

point KF::getState() const { return point(X[0], X[1], 0.); }

Gaussian<point, point> KF::getStdev() const
{
  Eigen::EigenSolver<Eigen::Matrix2f> s(P);
  Eigen::Vector2cf                    eigvec = s.eigenvectors().col(1);
  Eigen::Vector2cf                    eigval = s.eigenvalues();
  //  std::cout << "CHECK THE EIGENVECTOR!!! \n" << eigvec << std::endl;
  float angle = std::atan2(eigvec.real()[1], eigvec.real()[0]);

  point m_mean(X[0], X[1], X[2]);
  point m_stdev(eigval.real()[0], eigval.real()[1], 0.);
  return Gaussian<point, point>(m_mean, m_stdev, angle);
}

} // namespace vineslam
