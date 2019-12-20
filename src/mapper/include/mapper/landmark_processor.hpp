#pragma once

#include "utils.hpp"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
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

	cv::Mat projectToPlane(const cv::Mat&                   in,
	                       const std::vector<Line<double>>& trunks,
	                       const std::vector<Line<double>>& vine_lines,
	                       const Point<double>&             robot_p);

	std::vector<Match<double>>    matches;
	std::vector<Landmark<double>> landmarks;

  Line<double> vine_rline;
  Line<double> vine_lline;

private:
	Parameters                 params;
	std::vector<Point<double>> lc_pos;
	std::vector<Point<double>> lp_pos;
};
