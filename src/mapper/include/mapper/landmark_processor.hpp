#pragma once

#include "pf.hpp"
#include "utils.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>

class LandmarkProcessor
{
public:
	LandmarkProcessor(const Parameters& params);
	void updateDetections(const std::vector<Point<double>>& detections);
	void matchLandmarks(const int& iter, const Point<double>& r_pos,
	                    std::vector<int>& index);

	Line<double> computeLine(const Point<double>& landmark);
	Line<double> computeLine(const Point<double>& landmark, const double& phi);
	Line<double> projectLine(const Point<double>& pos,
	                         const Point<double>& delta_p,
	                         const double&        delta_th);

	std::vector<Match<double>>    matches;
	std::vector<Landmark<double>> landmarks;
	std::vector<PF>               pf;
	std::vector<KF>               kf;

private:
	Parameters                 params;
	std::vector<Point<double>> lc_pos;
	std::vector<Point<double>> lp_pos;
};
