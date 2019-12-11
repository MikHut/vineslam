#pragma once

#include "kf.hpp"
#include "utils.hpp"
#include <iostream>

class PF
{
public:
	PF(const bool& side, const Parameters& params);
	std::vector<Point<double>> process(const Pose<double>&  c_pose,
	                                   const Pose<double>&  p_pose,
	                                   const Point<double>& c_col,
	                                   const Point<double>& p_col,
	                                   const Line<double>&  vine);

	std::vector<Particle<double>> particles;

private:
	std::vector<Point<double>> predict(const Pose<double>&  c_pose,
	                                   const Pose<double>&  p_pose,
	                                   const Point<double>& c_col,
	                                   const Point<double>& p_col,
	                                   const Line<double>&  vine);

	void update();
	void resample();

	Point<double> expectEstimation(const Point<double>& X_prev,
	                               const Point<double>& p_pos, const double& dy);
	Point<double> processObsv(const Point<double>& c_col,
	                          const Point<double>& p_col,
	                          const Pose<double>&  delta_p);
	Line<double>  projectLine(const Point<double>& pos,
	                          const Point<double>& delta_p,
	                          const double&        delta_th);
	Line<double>  computeLine(const Point<double>& landmark);
	Line<double>  computeLine(const Point<double>& landmark, const double& phi);


	Parameters params;
	MatrixXd   P;
};
