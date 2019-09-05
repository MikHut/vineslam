#pragma once

#include "utils.hpp"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class LandmarkProcessor
{
public:
  LandmarkProcessor(int w, int h, int res);
  void    updatePoses(const std::vector<Point<int>> poses);
  cv::Mat buildGrid();

private:
  std::vector<Point<int>> landmark_poses;
  int			  width;
  int			  height;
  int			  resolution;
};
