#include "../../utils/utils.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>

#define PI 3.14159265359

const double std_y  = 25;
const double std_th = 4;

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KF
{
public:
	KF(const VectorXd& X0, const MatrixXd& P0, const Parameters& params);

	void     process(const VectorXd& s, const VectorXd& z);
	VectorXd getState() const;

private:
	void predict();
	void correct(const VectorXd& s, const VectorXd& z);
	void computeR(const VectorXd& s, const VectorXd& z);

	Parameters params;

	/* state vector and KF matrices */
	VectorXd X0;
	VectorXd X;
	MatrixXd P;
	MatrixXd K;
	MatrixXd R;

	/* auxiliar funcions */
	double normalizeAngle(const double& angle)
	{
		return (std::fmod(angle + PI, 2 * PI) - PI);
	}

	double sigmoid(const double& x, const bool& is_inside)
	{
		double y = x / (1 + std::fabs(x) * 10);
		return y * (!is_inside) * 1000;
	}
};
