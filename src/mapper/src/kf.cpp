#include "../include/mapper/kf.hpp"

KF::KF(const VectorXd& X0, const MatrixXd& P0, const Parameters& params)
    : X(X0), X0(X0), P(P0), params(params)
{
}

void KF::process(const VectorXd& s, const VectorXd& z)
{
	computeR(s, z);
	predict();
	correct(s, z);
}

void KF::computeR(const VectorXd& s, const VectorXd& z) {}

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
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(R);

	const VectorXd& eig_values(eig.eigenvalues());
	const MatrixXd& eig_vectors(eig.eigenvectors());

	double th    = (atan2(eig_vectors(1, 0), eig_vectors(0, 0)));
	double std_x = sqrt(eig_values[0]);
	double std_y = sqrt(eig_values[1]);

	return Ellipse<double>(std_x, std_y, th);
}
