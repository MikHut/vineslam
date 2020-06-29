#pragma once

// Class objects
#include <feature.hpp>
#include <occupancy_map.hpp>
#include <pf.hpp>
#include <math/point.hpp>
#include <math/pose.hpp>
#include <math/const.hpp>

// std, eigen
#include <iostream>
#include <map>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <yaml-cpp/yaml.h>

namespace vineslam
{

// Structure that stores the multi-layer mapping observations to use in the
// localization procedure
struct Observation {
  std::vector<SemanticFeature> landmarks;
  std::vector<Corner>          corners;
  std::vector<ImageFeature>    surf_features;
  Plane                        ground_plane;
};

class Localizer
{
public:
  // Class constructor
  Localizer(const std::string& config_path);

  // Initializes the particle filter with the number of particles
  // and the first odometry pose
  void init(const pose& initial_pose);

  // Global function that handles all the localization process
  // Arguments:
  // - odom:      wheel odometry pose
  // - obsv:      current multi-layer mapping observation
  // - grid_map:  occupancy grid map that encodes the multi-layer map information
  void process(const pose& odom, const Observation& obsv, const OccupancyMap& grid_map);

  // Export the final pose resultant from the localization procedure
  pose getPose() const;
  // Export the all the poses referent to all the particles
  void getParticles(std::vector<pose>& in) const;

private:
  // Average particles pose
  pose average_pose;
  // Particle filter object
  PF* pf{};
  // Input parameters
  float n_particles;
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
  static float normalizeAngle(const float& angle)
  {
    return static_cast<float>(std::fmod(angle + PI, 2 * PI) - PI);
  }
};
}; // namespace vineslam
