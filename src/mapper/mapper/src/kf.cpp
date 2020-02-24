#include "kf.hpp"

KF::KF(const VectorXd& X0, const MatrixXd& P0, const Parameters& params)
    : X(X0), X0(X0), P(P0), params(params)
{
	n_obsvs = 1;
	R       = P0;
}

void KF::process(const VectorXd& s, const VectorXd& z)
{
	n_obsvs++;

	computeR(s, z);
	predict();
	correct(s, z);
}

void KF::computeR(const VectorXd& s, const VectorXd& z)
{
	// Calculate observation (depth,bearing)
	double d   = sqrt(pow(X[0] - s[0], 2) + pow(X[1] - s[1], 2));
	double phi = atan2(X[1] - s[1], X[0] - s[0]) - s[2];

	// Odometry travelled distance
	double odom_std = sqrt(pow(z[0], 2) + pow(z[1], 2));

	// Compute the covariance observations matrix based on
	// - the distance from the detected trunk to the robot
	// - the travelled distance given by odometry
	// - the number of observations of the landmark
	R       = MatrixXd(2, 2);
	R(0, 0) = dispError(d) * std::fabs(cos(phi)) * (odom_std * 1.01);
	R(1, 1) = dispError(d) * std::fabs(sin(phi)) * (odom_std * 1.01);
	R(0, 1) = 0;
	R(1, 0) = 0;
}

void KF::predict()
{
	// Compute the state model - static
	X = X;
	P = P;
}

void KF::correct(const VectorXd& s, const VectorXd& z)
{
	// Apply the observation model using the current state vector
	double d   = sqrt(pow(X[0] - s[0], 2) + pow(X[1] - s[1], 2));
	double phi = atan2(X[1] - s[1], X[0] - s[0]) - s[2];

	VectorXd z_(2, 1);
	z_ << d, normalizeAngle(phi);

	// Compute the Jacobian of the non linear observation vector
	MatrixXd G(2, 2);
	G << (X[0] - s[0]) / d, +(X[1] - s[1]) / d, -(X[1] - s[1]) / pow(d, 2),
	    (X[0] - s[0]) / pow(d, 2);

	// Compute the Kalman gain
	K = P * G.transpose() * (G * P * G.transpose() + R).inverse();

	// Compute the difference between the observation vector and the
	// output of the observation model
	VectorXd z_diff = z - z_;
	z_diff[1]       = normalizeAngle(z_diff[1]);

	// Update the state vector and the process covariance matrix
	X = X + K * z_diff;
	P = (MatrixXd::Identity(2, 2) - K * G) * P;
}

Point<double> KF::getState() const
{
	return Point<double>(X[0], X[1]);
}

Ellipse<double> KF::getStdev() const
{
  std::cout << P << std::endl;
	double a = P(0, 0);
	double b = P(0, 1);
	double c = P(1, 1);

	double lambda_1 = (a + c) / 2 + sqrt(pow(a - c, 2) + pow(b, 2));
	double lambda_2 = (a + c) / 2 - sqrt(pow(a - c, 2) + pow(b, 2));

	double th;
	if (b == 0 && a >= c)
		th = 0;
	else if (b == 0 && a < c)
		th = PI / 2;
	else
		th = atan2(lambda_1 - a, b);

  double std_x = sqrt(lambda_1);
  double std_y = sqrt(std::fabs(lambda_2));

	return Ellipse<double>(std_x, std_y, th);
}
