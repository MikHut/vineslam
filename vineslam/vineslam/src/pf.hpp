#pragma once

// Include class objects
#include <feature.hpp>
#include <occupancy_map.hpp>
#include <icp.hpp>
#include <math/pose.hpp>
#include <math/stat.hpp>
#include <math/const.hpp>

// Include std members and yaml-cpp
#include <cstdlib>
#include <limits>
#include <chrono>
#include <iostream>
#include <map>
#include <yaml-cpp/yaml.h>

namespace vineslam
{

// Struct that represents a single particle with
// - identification number
// - 6-DOF pose
// - weight
struct Particle {
  Particle() = default;
  Particle(const int& id, const pose& p, const float& w)
  {
    (*this).id = id;
    (*this).p  = p;
    (*this).w  = w;
  }

  int   id{};
  pose  p;
  float w{};
  int   which_cluster{};
};

// Print particle ...
static std::ostream& operator<<(std::ostream& o, const Particle& p)
{
  o << "Particle " << p.id << ":\n" << p.p << p.w << "\n\n";
  return o;
}

class PF
{
public:
  // Class constructor
  // - initializes the total set of particles
  PF(const std::string& config_path,
     const pose&        initial_pose,
     const int&         m_n_particles);

  // Apply odometry motion model to all particles
  void motionModel(const pose& odom);
  // Update particles weights using the multi-layer map
  void update(const int&                          xmin,
              const int&                          xmax,
              const std::vector<SemanticFeature>& landmarks,
              const std::vector<Corner>&          corners,
              const Plane&                        ground_plane,
              const pose&                         gps_pose,
              OccupancyMap                        grid_map);
  // Normalize particles weights
  void normalizeWeights();
  // Resample particles
  void resample();
  // K-means based particle clustering
  void cluster(std::map<int, Gaussian<pose, pose>>& gauss_map);
  // Scan match on clustered particles
  void scanMatch(const std::map<int, Gaussian<pose, pose>>& gauss_map,
                 const std::vector<ImageFeature>&           features,
                 const OccupancyMap&                        grid_map);

  // Last iteration vars
  pose p_odom;

  // Particle weight sum
  float w_sum{};

  // Particles
  std::vector<Particle> particles;

private:
  // Iterative closest point member
  ICP *icp;

  // Input parameters file name
  std::string config_path;
  // Input numeric parameters
  int   n_particles;
  float cam_pitch;
  float srr;
  float str;
  float stt;
  float srt;
  float sigma_xy;
  float sigma_z;
  float sigma_roll;
  float sigma_pitch;
  float sigma_yaw;
  float sigma_landmark_matching;
  float sigma_feature_matching;
  float sigma_corner_matching;
  float sigma_ground_rp;
  float sigma_gps;
  int   k_clusters;
  int   k_iterations;
};

}; // namespace vineslam
