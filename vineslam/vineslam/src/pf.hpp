#pragma once

// Include class objects
#include <feature.hpp>
#include <occupancy_map.hpp>
#include <math/pose.hpp>
#include <math/stat.hpp>

// Include std members and yaml-cpp
#include <stdlib.h>
#include <limits>
#include <chrono>
#include <iostream>
#include <map>
#include <yaml-cpp/yaml.h>

#define PI 3.14159265359

namespace vineslam
{

// Struct that represents a single particle with
// - identification number
// - 6-DOF pose
// - weight
struct Particle {
  Particle() {}
  Particle(const int& id, const pose& p, const float& w)
  {
    (*this).id = id;
    (*this).p  = p;
    (*this).w  = w;
  }
  int   id;
  pose  p;
  float w;
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
  PF(const std::string& config_path, const pose& initial_pose);

  // Apply odometry motion model to a single pose
  void drawFromMotion(const pose& odom, pose& p) const;
  // Apply odometry motion model to all particles
  void motionModel(const pose& odom);
  // Update particles weights using the multi-layer map
  void update(const std::vector<SemanticFeature>& landmarks,
              const std::vector<Corner>&          corners,
              const Plane&                        ground_plane,
              OccupancyMap                        grid_map);
  // Normalize particles weights
  void normalizeWeights();
  // Resample particles
  void resample();

  // Previous odometry control
  pose p_odom;

  // Particle weight sum
  float w_sum{};

  // Particles
  std::vector<Particle> particles;

private:
  // Input parameters file name
  std::string config_path;
  // Input numeric parameters
  float cam_pitch;
  float srr;
  float str;
  float stt;
  float srt;
  float sigma_z;
  float sigma_roll;
  float sigma_pitch;
  float n_particles;
};

}; // namespace vineslam
