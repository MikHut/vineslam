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
#include <math/point.hpp>
#include <math/pose.hpp>

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
#define FAST_ 1
#define ORB_ 0
#define KAZE_ 0
#define AKAZE_ 0

#define PI 3.14159265359

namespace vineslam
{
class Mapper3D
{
public:
  // Class constructor - receives and saves the system
  // parameters
  Mapper3D(const std::string& config_path);

  // Built local map given the current observations
  void
  localMap(const cv::Mat& img, float* depths, std::vector<Feature>& out_features);

  // Adds the features to the global map
  void globalMap(const std::vector<Feature>& features,
                 const pose&                 robot_pose,
                 OccupancyMap&               grid_map);

private:
  // Perform feature extraction
  void featureHandler(const cv::Mat& in, std::vector<Feature>& out);

  // Converts a pixel into world's coordinate reference
  void pixel2world(const point& in_pt, const float& depth, point& out_pt) const;

  // Camera info parameters
  float img_width;
  float img_height;
  float cam_height;
  float cam_pitch;
  float fx;
  float fy;
  float cx;
  float cy;
  // 3D map parameters
  float       max_range;
  float       max_height;
  std::string fdetector;
};
}; // namespace vineslam
