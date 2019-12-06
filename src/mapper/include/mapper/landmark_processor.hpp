#pragma once

#include "pf.hpp"
#include "utils.hpp"
#include <iostream>

class LandmarkProcessor
{
public:
	LandmarkProcessor(const Parameters& params);
	void updateDetections(const std::vector<Point<double>>& detections);
	void matchLandmarks(const int& iter, std::vector<int>& index);

  Line<double> computeLine(const Point<double>& landmark);

	std::vector<Match<double>>    matches;
	std::vector<Landmark<double>> landmarks;
	std::vector<PF>               pf;

private:
	Parameters                 params;
	std::vector<Point<double>> lc_pos;
	std::vector<Point<double>> lp_pos;
};
