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
  // - Receives the initial state, the initial covarianec matrix, 
  //   and the parameters
	KF(const VectorXd& X0, const MatrixXd& P0, const Parameters& params);
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

	// Auxiliar funcion that normalizes an angle in the [-pi,pi] range
	double normalizeAngle(const double& angle)
	{
		return (std::fmod(angle + PI, 2 * PI) - PI);
	}
};
