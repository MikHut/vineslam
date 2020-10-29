#pragma once

// Class objects
#include <params.hpp>
#include <feature.hpp>
#include <occupancy_map.hpp>
#include <pf.hpp>
#include <math/point.hpp>
#include <math/pose.hpp>
#include <math/const.hpp>

// std, eigen
#include <iostream>
#include <thread>
#include <map>

namespace vineslam
{

// Structure that stores  observations to use in the localization procedure
struct Observation {
  std::vector<SemanticFeature> landmarks;
  std::vector<ImageFeature>    surf_features;
  std::vector<Planar>          planars;
  std::vector<Corner>          corners;
  std::vector<Plane>           planes;
  Plane                        ground_plane;
  pose                         gps_pose;
};

class Localizer
{
public:
  // Class constructor
  explicit Localizer(Parameters params);

  // Initializes the particle filter with the number of particles
  // and the first odometry pose
  void init(const pose& initial_pose);

  // Global function that handles all the localization process
  // Arguments:
  // - odom:      wheel odometry pose
  // - obsv:      current multi-layer mapping observation
  // - grid_map:  occupancy grid map that encodes the multi-layer map information
  void process(const pose&        odom,
               const Observation& obsv,
               OccupancyMap*      previous_map,
               OccupancyMap*      grid_map);

  // Export the final pose resultant from the localization procedure
  pose getPose() const;
  // Export the all the poses referent to all the particles
  void getParticles(std::vector<Particle>& in) const;
  void getParticlesBeforeResampling(std::vector<Particle>& in) const;

private:
  // Average particles pose
  pose average_pose;
  // Particle filter object
  PF* pf{};

  // Particles before resampling
  std::vector<Particle> m_particles;

  // Input parameters
  Parameters params;
};

} // namespace vineslam
