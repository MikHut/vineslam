#pragma once

#include "../../utils/utils.hpp"
#include <array>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class LandmarkProcessor
{
public:
	LandmarkProcessor(const Parameters& params);
	void         updatePoses(const std::vector<Point<double>>& poses);
	void         matchLandmarks(const int& iter);
	Line<double> projectLine(const Point<double>& pos,
	                         const Point<double>& delta_p,
	                         const double&        delta_th);
	Line<double> computeLine(const Point<double>& landmark);
	Line<double> computeLine(const Point<double>& landmark, const double& phi);

	std::vector<Match<double>>    matches;
	std::vector<Landmark<double>> landmarks;
	Grid<int>                     grid;

private:
	Parameters                 params;
	std::vector<Point<double>> lc_pose;
	std::vector<Point<double>> lp_pose;
};
