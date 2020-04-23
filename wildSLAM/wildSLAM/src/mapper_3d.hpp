#pragma once

#include <iostream>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <yaml-cpp/yaml.h>

// OCTOMAP
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Math
#include <math/point3D.hpp>
#include <math/pose6D.hpp>

// Classes
#include <feature.hpp>

#define PI 3.14159265359

// Typedefs
typedef octomap::ColorOcTree OcTreeT;

class Mapper3D
{
public:
  // Class constructor - receives and saves the system
  // parameters
  Mapper3D(const std::string& config_path);

  // Function that handles the design of the vine trunks 3D map
  void trunkMap(OcTreeT*                             octree,
                const float*                         depths,
                cv::Mat                              image,
                pose6D                               sensor_origin,
                const vision_msgs::Detection2DArray& dets);
  // Function that handles the design of the feature 3D map
  void featureMap(OcTreeT*                    octree,
                  const float*                depths,
                  cv::Mat                     image,
                  pose6D                      sensor_origin,
                  const std::vector<Feature>& features);

private:
  // Creates an OctoMap using the Octree structure
  void updateOctoMap(OcTreeT*                                   octree,
                     pose6D&                                    sensor_origin,
                     const std::vector<point3D>&                pcl,
                     const std::vector<std::array<uint8_t, 3>>& ints);

  // TODO (André Aguiar): Misses documentation for this function
  inline static void updateMinKey(const octomap::OcTreeKey& in,
                                  octomap::OcTreeKey&       min)
  {
    for (unsigned i = 0; i < 3; ++i) min[i] = std::min(in[i], min[i]);
  };

  // TODO (André Aguiar): Misses documentation for this function
  inline static void updateMaxKey(const octomap::OcTreeKey& in,
                                  octomap::OcTreeKey&       max)
  {
    for (unsigned i = 0; i < 3; ++i) max[i] = std::max(in[i], max[i]);
  };

  // Octree parameters
  float max_range;

  // Camera info parameters
  float img_width;
  float img_height;
  float cam_height;
  float fx;
  float fy;
  float cx;
  float cy;
};
