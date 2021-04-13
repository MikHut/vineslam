#pragma once

// Class objects
#include "../params.hpp"
#include "../feature/visual.hpp"
#include "../feature/semantic.hpp"
#include "../feature/three_dimensional.hpp"
#include "../mapping/occupancy_map.hpp"
#include "../matcher/icp.hpp"
#include "../localization/pf.hpp"
#include "../math/Point.hpp"
#include "../math/Pose.hpp"
#include "../math/const.hpp"

// std, eigen
#include <iostream>
#include <thread>
#include <map>

namespace vineslam
{
// Structure that stores  observations to use in the localization procedure
struct Observation
{
  std::vector<SemanticFeature> landmarks_;
  std::vector<ImageFeature> surf_features_;
  std::vector<Planar> planars_;
  std::vector<Corner> corners_;
  std::vector<SemiPlane> planes_;
  SemiPlane ground_plane_;
  Pose gps_pose_;
  Pose imu_pose_;
};

class Localizer
{
public:
  // Class constructor
  explicit Localizer(Parameters params);

  // Initializes the particle filter with the number of particles
  // and the first odometry pose
  void init(const Pose& initial_pose);

  // Global function that handles all the localization process
  // Arguments:
  // - wheel_odom_inc: odometry incremental pose
  // - obsv:           current multi-layer mapping observation
  // - grid_map:       occupancy grid map that encodes the multi-layer map information
  void process(const Pose& odom, const Observation& obsv, OccupancyMap* grid_map);

  // Export the final pose resultant from the localization procedure
  Pose getPose() const;
  // Export the all the poses referent to all the particles
  void getParticles(std::vector<Particle>& in) const;
  void getParticlesBeforeResampling(std::vector<Particle>& in) const;

  // Routine to change the observations to use in the localization procedure
  void changeObservationsToUse(const bool& use_semantic_features, const bool& use_lidar_features,
                               const bool& use_image_features, const bool& use_gps);

  // Localization logs
  std::string logs_;

private:
  // Average particles pose
  Pose average_pose_;
  Pose last_update_pose_;
  Pose p_odom_;
  // Particle filter object
  PF* pf_{};

  // Particles before resampling
  std::vector<Particle> m_particles_;

  // Flags
  bool init_flag_;

  // Input parameters
  Parameters params_;
};

}  // namespace vineslam
