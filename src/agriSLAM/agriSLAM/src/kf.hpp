#include "utils.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>

#define PI 3.14159265359

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KF
{
public:
	KF(){};
	// Class constructor
	// - Receives the initial state and the parameters
  // - initializes the covariance matrix
	KF(const VectorXd& X0, const VectorXd& s, const VectorXd& z,
	   const MatrixXd& R, const Parameters& params);
	// Function that processes all the Kalman Filter routines
	void process(const VectorXd& s, const VectorXd& z);
	// Function that outputs the current state of the Kalman Filter
	Point<double> getState() const;
	// Function that outputs the current standard deviation of the
	// Kalman Filter
	Ellipse<double> getStdev() const;

	MatrixXd P;

private:
	// Function that implements the prediction step of the Kalman Filter
	void predict();
	// Function that implements the update step of the Kalman Filter
	void correct(const VectorXd& s, const VectorXd& z);
	// Function that calculates the current observations covariance matrix
	void computeR(const VectorXd& s, const VectorXd& z);

	Parameters params;

	// State vector and KF matrices
	VectorXd X0;
	VectorXd X;
	MatrixXd K;
	MatrixXd R;

	// Number of observations of the landmark considered in the KF object
	int n_obsvs;

	// Auxiliar function that normalizes an angle in the [-pi,pi] range
	double normalizeAngle(const double& angle)
	{
		return (std::fmod(angle + PI, 2 * PI) - PI);
	}
	// Calculates the disparity error using the disparity noise model
	double dispError(const double& depth)
	{
		return pow(depth, 2) / (params.baseline * params.f_length) * params.delta_D;
	}
};
