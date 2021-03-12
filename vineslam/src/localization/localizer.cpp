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

void Localizer::process(const Pose& wheel_odom_inc, const Observation& obsv, OccupancyMap* previous_map,
                        OccupancyMap* grid_map)
{
  auto before = std::chrono::high_resolution_clock::now();
  // Resets
  pf_->w_sum_ = 0.;

  // ------------------------------------------------------------------------------
  // ---------------- Predict the robot motion
  // ------------------------------------------------------------------------------
  Pose odom_inc = wheel_odom_inc;
  odom_inc.normalize();

  // ------------------------------------------------------------------------------
  // ---------------- Draw particles using odometry motion model
  // ------------------------------------------------------------------------------
  pf_->motionModel(odom_inc, p_odom_);
  // - Save not resampled particles
  m_particles_.clear();
  for (const auto& particle : pf_->particles_)
    m_particles_.push_back(particle);

  Pose odom = p_odom_ + odom_inc;
  Pose delta_pose = odom - last_update_pose_;
  delta_pose.normalize();

  if (std::fabs(delta_pose.x_) > 0.1 || std::fabs(delta_pose.y_) > 0.1 ||
      std::fabs(delta_pose.Y_) > 2 * DEGREE_TO_RAD || init_flag_)
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

    last_update_pose_ = odom;
    init_flag_ = false;
  }

  // - Compute final robot pose using the mean of the particles poses
  std::vector<Pose> poses;
  for (const auto& particle : pf_->particles_)
    poses.push_back(particle.p_);
  average_pose_ = Pose(poses);

  // - Save current control to use in the next iteration
  p_odom_ = odom;

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

void Localizer::changeObservationsToUse(const bool& use_semantic_features, const bool& use_lidar_features,
                                        const bool& use_image_features, const bool& use_gps)
{
  pf_->use_semantic_features_ = use_semantic_features;
  pf_->use_lidar_features_ = use_lidar_features;
  pf_->use_image_features_ = use_image_features;
  pf_->use_gps_ = use_gps;
}

void Localizer::predictMotion(const Tf& initial_guess, const std::vector<Planar>& planars, OccupancyMap* previous_map,
                              Tf& result)
{
  // -------------------------------------------------------------------------------
  // ----- Planar features ICP
  // -------------------------------------------------------------------------------

  // - Set ICP input parameters
  ICP<Planar> planar_icp;
  planar_icp.setInputSource(planars);
  planar_icp.setInputTarget(previous_map);
  planar_icp.setTolerance(1e-5);
  planar_icp.setMaxIterations(50);
  planar_icp.setRejectOutliersFlag(false);

  // - Compute ICP
  float rms_error;
  std::vector<Planar> aligned;
  if (planar_icp.align(rms_error, aligned))
  {
    planar_icp.getTransform(result);
  }
  else
  {
    result = initial_guess;
  }

  std::vector<float> errors;
  planar_icp.getErrors(errors);
}

}  // namespace vineslam
