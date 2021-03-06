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

void Localizer::process(const Pose& wheel_odom_inc, const Observation& obsv, OccupancyMap* grid_map)
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
  pf_->motionModel(odom_inc);
  // - Save not resampled particles
  m_particles_.clear();
  for (const auto& particle : pf_->particles_)
    m_particles_.push_back(particle);

  Pose odom = p_odom_ + odom_inc;
  Pose delta_pose = odom - last_update_pose_;
  delta_pose.normalize();

  if (std::fabs(delta_pose.x_) > 0.15 || std::fabs(delta_pose.y_) > 0.15 ||
      std::fabs(delta_pose.Y_) > 3 * DEGREE_TO_RAD || init_flag_)
  {
    // ------------------------------------------------------------------------------
    // ---------------- Update particles weights using multi-layer map
    // ------------------------------------------------------------------------------
    pf_->update(obsv.landmarks_, obsv.corners_, obsv.planars_, obsv.planes_, obsv.ground_plane_, obsv.gps_pose_,
                obsv.imu_pose_, grid_map);

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

void Localizer::changeGPSFlag(const bool& val)
{
  pf_->use_gps_ = val;
}

}  // namespace vineslam
