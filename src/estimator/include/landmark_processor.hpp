#pragma once

#include "utils.hpp"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class LandmarkProcessor
{
public:
  LandmarkProcessor(const parameters& params);
  void    updatePoses(const std::vector<Point<int>> poses);
  cv::Mat buildVisualGrid();
  void    buildGrid();
  void    matchLandmarks();

  std::vector<Match<int>> matches;

private:
  std::vector<Point<int>> lc_pose;
  std::vector<Point<int>> lp_pose;
  parameters		  params;
};
