#include "localizer.hpp"

namespace vineslam
{

Localizer::Localizer(const std::string& config_path)
    : config_path(config_path)
{
  // Read input parameters
  YAML::Node config = YAML::LoadFile(config_path);
  img_width         = config["camera_info"]["img_width"].as<float>();
  img_height        = config["camera_info"]["img_height"].as<float>();
  cam_height        = config["camera_info"]["cam_height"].as<float>();
  fx                = config["camera_info"]["fx"].as<float>();
  fy                = config["camera_info"]["fy"].as<float>();
  cx                = config["camera_info"]["cx"].as<float>();
  cy                = config["camera_info"]["cy"].as<float>();
  n_particles       = config["pf"]["n_particles"].as<float>();
  use_gps           = config["system"]["use_gps"].as<bool>();
}

void Localizer::init(const pose& initial_pose)
{
  // Initialize the particle filter
  pf = new PF(config_path, initial_pose);

  // Compute average pose and standard deviation of the
  // first distribution
  std::vector<pose> poses;
  for (auto& particle : pf->particles) poses.push_back(particle.p);
  average_pose = pose(poses);
}

void Localizer::process(const pose&         odom,
                        const Observation&  obsv,
                        const OccupancyMap& grid_map)
{
  auto before = std::chrono::high_resolution_clock::now();
  // Reset weights sum
  pf->w_sum = 0.;

  // ------------------------------------------------------------------------------
  // ---------------- Draw particles using odometry motion model
  // ------------------------------------------------------------------------------
  pf->motionModel(odom);
  // ------------------------------------------------------------------------------
  // ---------------- Update particles weights using multi-layer map
  // ------------------------------------------------------------------------------
  if (!(obsv.gps_pose.x == 0. && obsv.gps_pose.y == 0. && obsv.gps_pose.z == 0. &&
        obsv.gps_pose.roll == 0. && obsv.gps_pose.pitch == 0. &&
        obsv.gps_pose.yaw == 0.))
    pf->update(
        obsv.landmarks, obsv.corners, obsv.ground_plane, obsv.gps_pose, grid_map);
  else
    pf->update(obsv.landmarks, obsv.corners, obsv.ground_plane, grid_map);
  // ------------------------------------------------------------------------------
  // ---------------- Normalize particle weights
  // ------------------------------------------------------------------------------
  pf->normalizeWeights();
  // ------------------------------------------------------------------------------
  // ---------------- Resample particles
  // ------------------------------------------------------------------------------
  pf->resample();

  // - Compute final robot pose using the mean of the particles poses
  std::vector<pose> poses;
  for (const auto& particle : pf->particles) poses.push_back(particle.p);
  average_pose = pose(poses);

  // - Save current control to use in the next iteration
  pf->p_odom = odom;
  auto after = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float, std::milli> duration = after - before;
  std::cout << "Time elapsed on PF (msecs): " << duration.count() << std::endl;
}

pose Localizer::getPose() const { return average_pose; }

void Localizer::getParticles(std::vector<pose>& in) const
{
  in.resize(pf->particles.size());
  for (size_t i = 0; i < in.size(); i++) in[i] = pf->particles[i].p;
}

} // namespace vineslam
