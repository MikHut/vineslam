#include "localizer.hpp"

Localizer::Localizer(const std::string& config_path)
    : config_path(config_path)
{
  // Read input parameters
  YAML::Node config = YAML::LoadFile(config_path.c_str());
  cam_pitch         = config["camera_info"]["cam_pitch"].as<float>() * PI / 180;
  img_width         = config["camera_info"]["img_width"].as<float>();
  img_height        = config["camera_info"]["img_height"].as<float>();
  cam_height        = config["camera_info"]["cam_height"].as<float>();
  fx                = config["camera_info"]["fx"].as<float>();
  fy                = config["camera_info"]["fy"].as<float>();
  cx                = config["camera_info"]["cx"].as<float>();
  cy                = config["camera_info"]["cy"].as<float>();
  n_particles       = config["pf"]["n_particles"].as<int>();
}

void Localizer::init(const pose6D& initial_pose)
{
  // Initialize the particle filter
  pf = new PF(config_path, n_particles, initial_pose);

  // Get the first distribution of particles
  std::vector<Particle> particles;
  (*pf).getParticles(particles);

  // Compute average pose and standard deviation of the
  // first distribution
  std::vector<pose6D> poses;
  for (size_t i = 0; i < particles.size(); i++) poses.push_back(particles[i].pose);
  average_pose = pose6D(poses);
}

void Localizer::process(const pose6D&                  odom,
                        const std::vector<float>&      bearings2D,
                        const std::vector<float>&      landmark_depths,
                        float*                         feature_depths,
                        OccupancyMap                   grid_map)
{
  // Invocate the particle filter loop
  (*pf).process(
      odom, bearings2D, landmark_depths, feature_depths, grid_map);
  // Import the resultant set of particles
  std::vector<Particle> particles;
  (*pf).getParticles(particles);

  // Compute the average pose and convert the particles pose to
  // ROS array
  std::vector<pose6D> m_poses;
  for (auto& particle : particles) {
    // Push back to the poses array
    pose6D m_pose = particle.pose;
    m_poses.push_back(m_pose);
  }
  average_pose = pose6D(m_poses);
  // Normalize average pose angles between [-pi,pi]
  average_pose.roll = normalizeAngle(average_pose.roll);
  // average_pose.pitch = cam_pitch + normalizeAngle(average_pose.pitch);
  average_pose.pitch = normalizeAngle(average_pose.pitch);
  average_pose.yaw   = normalizeAngle(average_pose.yaw);
}

pose6D Localizer::getPose() const { return average_pose; }

void Localizer::getParticles(std::vector<pose6D>& in) const
{
  // Get particles and resize input vector
  std::vector<Particle> particles;
  (*pf).getParticles(particles);
  in.resize(particles.size());

  for (size_t i = 0; i < in.size(); i++) in[i] = particles[i].pose;
}
