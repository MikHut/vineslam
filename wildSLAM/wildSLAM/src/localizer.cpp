#include "localizer.hpp"

namespace wildSLAM
{

Localizer::Localizer(const std::string& config_path)
    : config_path(config_path)
{
  // Read input parameters
  YAML::Node config = YAML::LoadFile(config_path.c_str());
  img_width         = config["camera_info"]["img_width"].as<float>();
  img_height        = config["camera_info"]["img_height"].as<float>();
  cam_height        = config["camera_info"]["cam_height"].as<float>();
  fx                = config["camera_info"]["fx"].as<float>();
  fy                = config["camera_info"]["fy"].as<float>();
  cx                = config["camera_info"]["cx"].as<float>();
  cy                = config["camera_info"]["cy"].as<float>();
  n_particles2D     = config["pf"]["n_particles2D"].as<float>();
  n_particles3D     = config["pf"]["n_particles3D"].as<float>();
}

void Localizer::init(const pose& initial_pose)
{
  // Initialize the particle filter
  pf = new PF(config_path, initial_pose);

  // Get the first distribution of particles
  std::vector<Particle> particles;
  pf->getParticles3D(particles);

  // Compute average pose and standard deviation of the
  // first distribution
  std::vector<pose> poses;
  for (auto& particle : particles) poses.push_back(particle.p);
  average_pose = pose(poses);
}

void Localizer::process(const pose&                  odom,
                        const std::vector<Landmark>& landmarks,
                        const std::vector<Feature>&  features,
                      OccupancyMap&          grid_map)
{
  // Invocate the particle filter loop
  pf->process(odom, landmarks, features, grid_map);
  // Import the resultant set of particles
  std::vector<Particle> particles;
  pf->getParticles3D(particles);

  // Compute the average pose and convert the particles pose to
  // ROS array
  std::vector<pose> m_poses;
  for (auto& particle : particles) {
    // Push back to the poses array
    pose m_pose = particle.p;
    m_poses.push_back(m_pose);
  }
  average_pose = pose(m_poses);
  // Normalize average pose angles between [-pi,pi]
  average_pose.roll  = normalizeAngle(average_pose.roll);
  average_pose.pitch = normalizeAngle(average_pose.pitch);
  average_pose.yaw   = normalizeAngle(average_pose.yaw);
}

pose Localizer::getPose() const { return average_pose; }

void Localizer::getParticles(std::vector<pose>& in) const
{
  // Get particles and resize input vector
  std::vector<Particle> particles;
  pf->getParticles3D(particles);
  in.resize(particles.size());

  for (size_t i = 0; i < in.size(); i++) in[i] = particles[i].p;
}

}; // namespace wildSLAM
