#pragma once

// Include class objects
#include <feature.hpp>
#include <occupancy_map.hpp>
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

  Particle(const int& id, const pose& p, const pose& last_p, const float& w)
  {
    (*this).id     = id;
    (*this).p      = p;
    (*this).last_p = last_p;
    (*this).w      = w;
  }

  int   id{};
  pose  p;
  pose  last_p;
  float w{};
};

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

  // Last iteration vars
  float last_ground_plane_z;
  pose  p_odom;

  // Particle weight sum
  float w_sum{};

  // Particles
  std::vector<Particle> particles;

  // Normalize an angle between -PI and PI
  static float normalizeAngle(const float& angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

private:
  // Input parameters file name
  std::string config_path;
  // Input numeric parameters
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
  float sigma_ground_z;
  float sigma_ground_rp;
  float sigma_gps;
  float semantic_norm;
  float corners_norm;
  float ground_norm;
  float gps_norm;
  int   n_particles;
};

}; // namespace vineslam
