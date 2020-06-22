#pragma once

// Include class objects
#include <feature.hpp>
#include <occupancy_map.hpp>
#include <icp.hpp>
#include <math/pose.hpp>
#include <math/stat.hpp>

// Include std members and yaml-cpp
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include <map>
#include <yaml-cpp/yaml.h>

#define PI 3.14159265359
#define INF 1e6

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

  // Global function that handles all the particle filter processes
  void process(const pose&                         odom,
               const std::vector<SemanticFeature>& landmarks,
               const std::vector<ImageFeature>&    features,
               OccupancyMap&                       grid_map);

  // Export the array of particles
  void getParticles(std::vector<Particle>& in);

private:
  // Apply odometry motion model to a single pose
  void drawFromMotion(const pose& odom, pose& p);
  // Apply odometry motion model to all particles
  void motionModel(const pose& odom);
  // Normalize particles weights
  void normalizeWeights();
  // Resample particles
  void resample(std::vector<Particle>& particles);

  // Previous odometry control
  pose p_odom;

  // Particle weight sum
  float w_sum;

  // Particles
  std::vector<Particle> particles3D;

  // Arrays to store the scan matcher rms error and gaussians
  std::vector<float>                  rms_error3D;
  std::vector<Gaussian<float, float>> dprobvec;
  std::vector<Gaussian<float, float>> sprobvec;

  // Scan Matcher objects
  ICP* icp;

  // Input parameters file name
  std::string config_path;
  // Input numeric parameters
  float srr;
  float str;
  float stt;
  float srt;
  float sigma_z;
  float sigma_roll;
  float sigma_pitch;
  float n_particles2D;
  float n_particles3D;
};

}; // namespace vineslam
