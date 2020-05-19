#pragma once

// std & ROS
#include <iostream>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <yaml-cpp/yaml.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Objects
#include <occupancy_map.hpp>
#include <math/point3D.hpp>
#include <math/pose6D.hpp>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <feature.hpp>

// Feature extractors supported
#define STAR_ 0
#define BRISK_ 0
#define FAST_ 0
#define ORB_ 1
#define KAZE_ 0
#define AKAZE_ 0

#define PI 3.14159265359

namespace wildSLAM
{
class Mapper3D
{
public:
  // Class constructor - receives and saves the system
  // parameters
  Mapper3D(const std::string& config_path);

  // Perform feature extraction
  void extractFeatures(const cv::Mat& in, std::vector<Feature>& out);

  // Built local map given the current observations
  void localMap(const std::vector<Feature>& in_features,
                std::vector<Feature>&       out_features);

  // Adds the features to the global map
  void globalMap(const std::vector<Feature>& features, OccupancyMap& grid_map);

private:
  // Camera info parameters
  float img_width;
  float img_height;
  float cam_height;
  float fx;
  float fy;
  float cx;
  float cy;
  // 3D map parameters
  float max_range;
  float max_height;
};
}; // namespace wildSLAM
