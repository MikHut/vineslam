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
  cv::Mat plotGrid();

  std::vector<Match<double>> matches;

private:
  Parameters		     params;
  std::vector<Point<double>> lc_pose;
  std::vector<Point<double>> lp_pose;

  double x_start;
  double x_end;

  void	 updatePoses(const std::vector<Point<double>>& poses);
  void	 matchLandmarks();
  Line<double> computeLine(const Point<double>& landmark);
  Line<double> projectLine(const Line<double>& l, const Point<double>& delta);
};
