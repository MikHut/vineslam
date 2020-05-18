#pragma once

// Class objects
#include <feature.hpp>
#include <landmark.hpp>
#include <occupancy_map.hpp>
#include <mapper_3d.hpp>
#include <math/point3D.hpp>
#include <math/pose6D.hpp>
#include <pf.hpp>

// std, eigen
#include <iostream>
#include <map>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <yaml-cpp/yaml.h>

#define PI 3.14159265359

class Localizer
{
public:
  // Class constructor
  Localizer(const std::string& config_path);

  // Initializes the particle filter with the number of particles
  // and the first odometry pose
  void init(const pose6D& initial_pose);

  // Global function that handles all the localization process
  // Arguments:
  // - odom:            wheel odometry pose6D
  // - bearings2D:      bearing observations to process 2D localization
  // - landmark_depths: depth observations to process 2D localization
  // - feature_depths:  raw sensor depths to process 3D localization
  // - grid_map:        occupancy grid map that encodes the multi-layer map
  // information
  void process(const pose6D&                  odom,
               const std::vector<float>&      bearings2D,
               const std::vector<float>&      landmark_depths,
               float*                         feature_depths,
               OccupancyMap                   grid_map);

  // Export the final pose resultant from the localization procedure
  pose6D getPose() const;
  // Export the all the poses referent to all the particles
  void getParticles(std::vector<pose6D>& in) const;

private:
  // Average particles pose
  pose6D average_pose;
  // Particle filter object
  PF* pf;
  // Input parameters
  int   n_particles;
  float cam_pitch;
  float img_width;
  float img_height;
  float cam_height;
  float fx;
  float fy;
  float cx;
  float cy;

  // Input parameters file name
  std::string config_path;

  // Auxiliar function that normalizes an angle in the [-pi,pi] range
  float normalizeAngle(const float& angle)
  {
    return (std::fmod(angle + PI, 2 * PI) - PI);
  }
};
