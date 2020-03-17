#include "kf.hpp"

KF::KF(const VectorXd& X0, const VectorXd& s, const VectorXd& g,
       const VectorXd& z, const Parameters& params)
    : X0(X0), X(X0), params(params)
{
	// Apply the observation model using the current state vector
	double d   = sqrt(pow(X[0] - s[0], 2) + pow(X[1] - s[1], 2));
	double phi = atan2(X[1] - s[1], X[0] - s[0]) - s[2];

	// Compute the Jacobian of the non linear observation vector
	MatrixXd G(2, 2);
	G << (X[0] - s[0]) / d, +(X[1] - s[1]) / d, -(X[1] - s[1]) / pow(d, 2),
	    (X[0] - s[0]) / pow(d, 2);

	// Initialize the process covariance P
	computeR(s, g, z);
	P = R;
}

void KF::process(const VectorXd& s, const VectorXd& g, const VectorXd& z)
{
	n_obsvs++;

	computeR(s, g, z);
	predict();
	correct(s, z);
}

void KF::computeR(const VectorXd& s, const VectorXd& g, const VectorXd& z)
{
	// Compute the covariance observations matrix based on
	// - more noise in depth to far observed landmarks
	// - less noise in detection for near observed landmarks
	// - the standard deviation of the particle filter pose
	//   estimation in both x and y components
	R       = MatrixXd(2, 2);
	R(0, 0) = dispError(z[0]) + g[0];
	R(1, 1) = 0.1 / (z[0] * z[0]) + g[1];
	R(0, 1) = 0;
	R(1, 0) = 0;

	// Rotate covariance matrix using the bearing angle
	// codified in a rotation matrix
	Eigen::MatrixXd Rot(2, 2);
	Rot << cos(z[1]), -sin(z[1]), sin(z[1]), cos(z[1]);

	R = Rot * R * Rot.transpose();
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
	double a = P(0, 0);
	double b = P(0, 1);
	double c = P(1, 1);

	double lambda_1 = (a + c) / 2 + sqrt(pow((a - c) / 2, 2) + pow(b, 2));
	double lambda_2 = (a + c) / 2 - sqrt(pow((a - c) / 2, 2) + pow(b, 2));

	double th;
	if (b == 0 && a >= c)
		th = 0;
	else if (b == 0 && a < c)
		th = PI / 2;
	else
		th = atan2(lambda_1 - a, b);

	// double std_x = sqrt(lambda_1);
	// double std_y = sqrt(lambda_2);
	double std_x = sqrt(a);
	double std_y = sqrt(c);

	return Ellipse<double>(std_x, std_y, 0);
}
