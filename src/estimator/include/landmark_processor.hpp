#pragma once

#include "utils.hpp"
#include <array>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class LandmarkProcessor
{
public:
	LandmarkProcessor(const Parameters& params);
	void process(const std::vector<Point<double>>& poses,
	             std::vector<Particle<double>>&    particles);

	/* Only for visualization */
	void plotGrid(const Line<double>& l, const int& color);
  void plotPMap(Point<double>& p, const int& color);
  cv::Mat grid;
  cv::Mat p_map;

	std::vector<Match<double>> matches;

private:
	Parameters                 params;
	std::vector<Point<double>> lc_pose;
	std::vector<Point<double>> lp_pose;

	double x_start;
	double x_end;

	void         updatePoses(const std::vector<Point<double>>& poses);
	void         matchLandmarks();
	Line<double> computeLine(const Point<double>& landmark);
	Line<double> computeLine(const Point<double>& landmark, const double& phi);
	Line<double> projectLine(const Match<double>& m, const Point<double>& delta_p,
	                         const double& delta_th);
};
