#pragma once

// Include class objects
#include "../params.hpp"
#include "../feature/visual.hpp"
#include "../feature/semantic.hpp"
#include "../feature/three_dimensional.hpp"
#include "../mapping/occupancy_map.hpp"
#include "../matcher/icp.hpp"
#include "../math/point.hpp"
#include "../math/pose.hpp"
#include "../math/const.hpp"
#include "../math/stat.hpp"


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
struct Particle {
  Particle() = default;
  Particle(const int& id, const pose& p, const float& w)
  {
    (*this).id = id;
    (*this).p  = p;
    (*this).w  = w;

    std::array<float, 9> Rot{};
    p.toRotMatrix(Rot);
    std::array<float, 3> trans = {p.x, p.y, p.z};
    tf                         = TF(Rot, trans);
  }

  int   id{};
  pose  p;   // pose
  pose  pp;  // previous pose
  TF    tf;  // pose resented as an homogeneous transformation
  TF    ptf; // previous homogeneous transformation
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
  PF(const Parameters& params, const pose& initial_pose);

  // Apply odometry motion model to all particles
  void motionModel(const pose& odom_inc, const pose& p_odom);
  // Update particles weights using the multi-layer map
  void update(const std::vector<SemanticFeature>& landmarks,
              const std::vector<Corner>&          corners,
              const std::vector<Planar>&          planars,
              const std::vector<Plane>&           planes,
              const Plane&                        ground_plane,
              const std::vector<ImageFeature>&    surf_features,
              const pose&                         gps_pose,
              OccupancyMap*                       grid_map);
  // Normalize particles weights
  void normalizeWeights();
  // Resample particles
  void resample();

  // Particle weight sum
  float w_sum{};

  // Particles
  std::vector<Particle> particles;

  // Logs
  std::string logs;

  // Observations to use
  bool use_landmarks;
  bool use_corners;
  bool use_planars;
  bool use_icp;
  bool use_gps;

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
  // - Medium level planar features layer
  void mediumLevelPlanars(const std::vector<Planar>& planars,
                          OccupancyMap*              grid_map,
                          std::vector<float>&        ws);
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
  ICP<ImageFeature>* icp;

  // Parameters structure
  Parameters params;
};

} // namespace vineslam
