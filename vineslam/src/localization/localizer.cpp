#include "../../include/vineslam/localization/localizer.hpp"

#include <utility>

namespace vineslam
{
Localizer::Localizer(Parameters params) : params_(std::move(params))
{
}

void Localizer::init(const Pose& initial_pose)
{
  // Initialize the particle filter
  pf_ = new PF(params_, initial_pose);

  // Compute average pose and standard deviation of the
  // first distribution
  std::vector<Pose> poses;
  for (auto& particle : pf_->particles_)
    poses.push_back(particle.p_);
  average_pose_ = Pose(poses);
  last_update_pose_ = initial_pose;

  p_odom_ = initial_pose;
  init_flag_ = true;
}

void Localizer::process(const Pose& odom, const Observation& obsv, OccupancyMap* previous_map, OccupancyMap* grid_map)
{
  auto before = std::chrono::high_resolution_clock::now();
  // Resets
  pf_->w_sum_ = 0.;

  // ------------------------------------------------------------------------------
  // ---------------- Compute LiDAR odometry to predict the robot motion
  // ------------------------------------------------------------------------------
  Tf tf;
  predictMotion(obsv.planars, previous_map, tf);
  Pose odom_inc(tf.R_array_, tf.t_array_);
  odom_inc.normalize();

  // ------------------------------------------------------------------------------
  // ---------------- Draw particles using odometry motion model
  // ------------------------------------------------------------------------------
  pf_->motionModel(odom_inc, p_odom_);
  // - Save not resampled particles
  m_particles_.clear();
  for (const auto& particle : pf_->particles_)
    m_particles_.push_back(particle);

  Pose icp_odom = p_odom_ + odom_inc;
  Pose delta_pose = icp_odom - last_update_pose_;
  delta_pose.normalize();

  if (std::fabs(delta_pose.x_) > 0.1 || std::fabs(delta_pose.y_) > 0.1 ||
      std::fabs(delta_pose.Y_) > 5 * DEGREE_TO_RAD || init_flag_)
  {
    // ------------------------------------------------------------------------------
    // ---------------- Update particles weights using multi-layer map
    // ------------------------------------------------------------------------------
    pf_->update(obsv.landmarks, obsv.corners, obsv.planars, obsv.planes, obsv.ground_plane, obsv.surf_features,
                obsv.gps_pose, grid_map);

    // ------------------------------------------------------------------------------
    // ---------------- Normalize particle weights
    // ------------------------------------------------------------------------------
    pf_->normalizeWeights();

    // ------------------------------------------------------------------------------
    // ---------------- Resample particles
    // ------------------------------------------------------------------------------
    pf_->resample();

    last_update_pose_ = icp_odom;
    init_flag_ = false;
  }

  // - Compute final robot pose using the mean of the particles poses
  std::vector<Pose> poses;
  for (const auto& particle : pf_->particles_)
    poses.push_back(particle.p_);
  average_pose_ = Pose(poses);

  // - Save current control to use in the next iteration
  p_odom_ = icp_odom;

  // - Save pf logs
  auto after = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float, std::milli> duration = after - before;

  logs_ = pf_->logs_ + "Time elapsed on PF (msecs): " + std::to_string(duration.count()) + "\n\n";
}

Pose Localizer::getPose() const
{
  return average_pose_;
}

void Localizer::getParticles(std::vector<Particle>& in) const
{
  in.resize(pf_->particles_.size());
  for (size_t i = 0; i < in.size(); i++)
    in[i] = pf_->particles_[i];
}

void Localizer::getParticlesBeforeResampling(std::vector<Particle>& in) const
{
  in.resize(m_particles_.size());
  for (size_t i = 0; i < in.size(); i++)
    in[i] = m_particles_[i];
}

void Localizer::changeObservationsToUse(const bool& use_high_level, const bool& use_corners, const bool& use_planars,
                                        const bool& use_icp, const bool& use_gps)
{
  pf_->use_landmarks_ = use_high_level;
  pf_->use_corners_ = use_corners;
  pf_->use_planars_ = use_planars;
  pf_->use_icp_ = use_icp;
  pf_->use_gps_ = use_gps;
}

void Localizer::predictMotion(const std::vector<Planar>& planars, OccupancyMap* previous_map, Tf& result)
{
  // -------------------------------------------------------------------------------
  // ----- Planar features ICP
  // -------------------------------------------------------------------------------

  // - Set ICP input parameters
  ICP<Planar> planar_icp(params_);
  planar_icp.setInputSource(planars);
  planar_icp.setInputTarget(previous_map);
  planar_icp.setTolerance(1e-5);
  planar_icp.setMaxIterations(50);
  planar_icp.setRejectOutliersFlag(false);

  // - Compute ICP
  float rms_error;
  std::vector<Planar> aligned;
  planar_icp.align(rms_error, aligned);
  planar_icp.getTransform(result);

  std::vector<float> errors;
  planar_icp.getErrors(errors);
}

}  // namespace vineslam
