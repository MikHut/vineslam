#pragma once

// Include class objects
#include <vineslam/params.hpp>
#include <vineslam/feature/semantic.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Const.hpp>
#include <vineslam/math/Stat.hpp>
#include <vineslam/filters/convex_hull.hpp>
#include <vineslam/filters/ransac.hpp>
#include <vineslam/utils/Timer.hpp>
#include <vineslam/extern/thread_pool.h>

// Include std members
#include <cstdlib>
#include <limits>
#include <chrono>
#include <iostream>
#include <map>
#include <cmath>

namespace vineslam
{
// Struct that represents a single particle with
// - identification number
// - 6-DOF pose
// - weight
struct Particle
{
  Particle() = default;
  Particle(const int& id, const Pose& p, const float& w)
  {
    (*this).id_ = id;
    (*this).p_ = p;
    (*this).w_ = w;

    std::array<float, 9> Rot{};
    p.toRotMatrix(Rot);
    std::array<float, 3> trans = { p.x_, p.y_, p.z_ };
    tf_ = Tf(Rot, trans);
  }

  int id_{};
  Pose p_;   // pose
  Pose pp_;  // previous pose
  Tf tf_;    // pose resented as an homogeneous transformation
  Tf ptf_;   // previous homogeneous transformation
  float w_{};
  int which_cluster_{};
};

class PF
{
public:
  // Class constructor
  // - initializes the total set of particles
  PF(const Parameters& params, const Pose& initial_pose);

  // Apply odometry motion model to all particles
  void motionModel(const Pose& odom_inc);
  // Update particles weights using the multi-layer map
  void update(const std::vector<SemanticFeature>& landmarks, const std::vector<Corner>& corners,
              const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes, const SemiPlane& ground_plane,
               const Pose& gps_pose, const Pose& imu_pose,
              OccupancyMap* grid_map);
  // Update 'beam model of range finders'
  void updateModel(const float& z_k, const float& z_k_asterisc, const float& z_dist, const float& sigma_hit,
                   const float& sigma_short, float& w);
  // Normalize particles weights
  void normalizeWeights();
  // Resample particles
  void resample();

  // Particle weight sum
  float w_sum_{};

  // Particles
  std::vector<Particle> particles_;

  // Thread pool
  lama::ThreadPool* thread_pool_;

  // Profiler
  Timer* t_;

  // Observations to use
  bool use_semantic_features_;
  bool use_lidar_features_;
  bool use_gps_;
  bool use_gps_altitude_;
  bool use_imu_;

  // Filter settings
  float sigma_landmark_matching_;
  float sigma_feature_matching_;
  float sigma_corner_matching_;
  float sigma_planar_matching_;
  float sigma_plane_matching_vector_;
  float sigma_plane_matching_centroid_;
  float sigma_gps_;
  float sigma_imu_;

private:
  // Samples a zero-mean gaussian distribution with a given standard deviation
  float sampleGaussian(const float& sigma, const unsigned long int& S = 0);

  // Update functions
  // - High level semantic features layer
  void highLevel(const std::vector<SemanticFeature>& landmarks, OccupancyMap* grid_map, std::vector<float>& ws);
  // - Medium level corner features layer
  void mediumLevelCorners(const std::vector<Corner>& corners, OccupancyMap* grid_map, std::vector<float>& ws);
  // - Medium level planar features layer
  void mediumLevelPlanars(const std::vector<Planar>& planars, OccupancyMap* grid_map, std::vector<float>& ws);
  // - Medium ground plane layer
  void mediumLevelPlanes(const std::vector<SemiPlane>& planes, OccupancyMap* grid_map, std::vector<float>& ws);
  // - GPS
  void gps(const Pose& gps_pose, std::vector<float>& ws);
  // - IMU
  void imu(const Pose& imu_pose, std::vector<float>& ws);

  // Number of particles
  uint32_t particles_size_;

  // Parameters structure
  Parameters params_;
};

}  // namespace vineslam
