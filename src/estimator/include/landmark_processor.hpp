#pragma once

#include "utils.hpp"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class LandmarkProcessor
{
public:
  LandmarkProcessor(const Parameters& params);
  void    updatePoses(const std::vector<Point<double>>& poses);
  void    matchLandmarks();
  cv::Mat plotGrid();

  std::vector<Match<double>> matches;
  std::vector<Point<double>> projectLine(const std::vector<Point<double>>& l,
					 const int&			   particle_id);

private:
  std::vector<Point<double>> lc_pose;
  std::vector<Point<double>> lp_pose;
  Parameters		     params;

  std::vector<Point<double>> computeLine(const Point<double>& landmark);
  std::vector<Point<double>> computeLine(const Point<double>& landmark,
					 const double&	orientation);
};
