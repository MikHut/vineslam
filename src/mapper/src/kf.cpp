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

void KF::computeR(const VectorXd& s, const VectorXd& z)
{
	double x  = z[0] * cos(z[1]) + s[0];
	double y  = z[1] * sin(z[1]) + s[1];
	double th = normalizeAngle(z[1]);

	VectorXd dt(2, 1);
	dt << s[0] - x, s[1] - y;

	double abs_min_thtol = -params.h_fov / 2 + s[2];
	double abs_max_thtol = +params.h_fov / 2 + s[2];
	bool is_inside = !(th < abs_min_thtol || th > abs_max_thtol || dt.norm() > 4);

	double dist_y  = y - X0[1];
	double dist_th = normalizeAngle(atan2(y, x) - atan2(X0[1], X0[0]));

	R = MatrixXd(2, 2);
	R << (!is_inside) * 1000 + std_y * pow(dist_y, 2), 0, 0,
	    (!is_inside) * 1000 + std_th * pow(dist_th, 2);
}

void KF::predict()
{
	X = X;
	P = P;
}

void KF::correct(const VectorXd& s, const VectorXd& z)
{
	double d   = sqrt(pow(X[0] - s[0], 2) + pow(X[1] - s[1], 2));
	double phi = atan2(X[1] - s[1], X[0] - s[0]) - s[2];

	VectorXd z_(2, 1);
	z_ << d, normalizeAngle(phi);

	MatrixXd G(2, 2);
	G << (X[0] - s[0]) / d, +(X[1] - s[1]) / d, -(X[1] - s[1]) / pow(d, 2),
	    (X[0] - s[0]) / pow(d, 2);

	K = P * G.transpose() * (G * P * G.transpose() + R).inverse();

	VectorXd z_diff = z - z_;
	z_diff[1]       = normalizeAngle(z_diff[1]);

	X = X + K * z_diff;
	P = (MatrixXd::Identity(2, 2) - K * G) * P;
}

VectorXd KF::getState() const
{
	return X;
}
