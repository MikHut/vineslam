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

	std::vector<Match<double>>
	matchLandmarks(const std::vector<Point<double>>& l_det,
	               const std::vector<Point<double>>& r_det);

	Line<double> computeLine(const Point<double>& landmark);
	Line<double> computeLine(const Point<double>& landmark, const double& phi);
	Line<double> projectLine(const Point<double>& pos,
	                         const Point<double>& delta_p,
	                         const double&        delta_th);


	cv::Mat stitchImgs(cv::Mat img_left, cv::Mat img_right);

	std::vector<Point<double>>
	projectToPlane(const cv::Mat& in, const std::vector<Line<double>>& trunks,
	               const std::vector<Line<double>>& vine_lines,
	               const Point<double>&             robot_p);

private:
	Parameters                 params;
	std::vector<Point<double>> lc_pos;
	std::vector<Point<double>> lp_pos;

	cv::Mat img_left;
	cv::Mat img_right;
};
