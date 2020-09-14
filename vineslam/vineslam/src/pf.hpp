#pragma once

// Include class objects
#include <feature.hpp>
#include <occupancy_map.hpp>
#include <icp.hpp>
#include <math/pose.hpp>
#include <math/stat.hpp>
#include <math/tf.hpp>
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

enum MOTION_STATE { FORWARD, ROTATING, STOPED };

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
  PF(const std::string& config_path, const pose& initial_pose);

  // Apply odometry motion model to all particles
  void motionModel(const pose& odom);
  // Update particles weights using the multi-layer map
  void update(const std::vector<SemanticFeature>& landmarks,
              const std::vector<Corner>&          corners,
              const std::vector<Line>&            vegetation_lines,
              const Plane&                        ground_plane,
              const std::vector<ImageFeature>&    surf_features,
              const pose&                         gps_pose,
              OccupancyMap*                       grid_map);
  // Normalize particles weights
  void normalizeWeights();
  // Resample particles
  void resample();

  // Last iteration vars
  pose p_odom;

  // Particle weight sum
  float w_sum{};

  // Particles
  std::vector<Particle> particles;

private:
  // Update functions
  // - High level semantic features layer
  void highLevel(const std::vector<SemanticFeature>& landmarks,
                 OccupancyMap*                       grid_map,
                 std::vector<float>&                 ws);
  // - Medium level corner features layer
  void mediumLevelCorners(const std::vector<Corner>& corners,
                          OccupancyMap*              grid_map,
                          std::vector<float>&        ws);
  // - Medium level ground plane layer
  void mediumLevelGround(const Plane& ground_plane, std::vector<float>& ws);
  // - Medium level vegetation lines layer
  void mediumLevelLines(const std::vector<Line>& vegetation_lines,
                        std::vector<float>&      ws);
  // - Low level image features layer
  void lowLevel(const std::vector<ImageFeature>& surf_features,
                OccupancyMap*                    grid_map,
                std::vector<float>&              ws);
  // -------- (Low level) K-means based particle clustering
  void cluster(std::map<int, Gaussian<pose, pose>>& gauss_map);
  // -------- (Low level) Scan match on clustered particles
  void scanMatch(const std::vector<ImageFeature>&     features,
                 OccupancyMap*                        grid_map,
                 std::map<int, Gaussian<pose, pose>>& gauss_map,
                 std::vector<float>&                  ws);
  // - GPS
  void gps(const pose& gps_pose, std::vector<float>& ws);

  // Iterative closest point member
  ICP* icp;

  // Iteration number
  int n_it;

  // Motion state
  MOTION_STATE motion_state;

  // Input parameters file name
  std::string config_path;
  // Input numeric parameters
  // - General parameters
  bool use_landmarks;
  bool use_corners;
  bool use_vegetation_lines;
  bool use_ground_plane;
  bool use_icp;
  bool use_gps;
  int  n_particles;
  // - Innovation parameters
  float srr;
  float str;
  float stt;
  float srt;
  float sigma_xy;
  float sigma_z;
  float sigma_roll;
  float sigma_pitch;
  float sigma_yaw;
  // - Update standard deviation of each layers
  float sigma_landmark_matching;
  float sigma_feature_matching;
  float sigma_corner_matching;
  float sigma_vegetation_lines_yaw;
  float sigma_ground_rp;
  float sigma_gps;
  // - Clustering parameters
  int k_clusters;
  int k_iterations;
};

} // namespace vineslam
