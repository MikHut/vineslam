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
  void    updatePoses(const std::vector<Point<double>> poses);
  cv::Mat plotGrid();
  void    buildGrid();
  void    matchLandmarks();

  std::vector<Match<double>> matches;

private:
  std::vector<Point<double>> lc_pose;
  std::vector<Point<double>> lp_pose;
  Parameters		  params;
};
