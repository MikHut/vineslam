#include "localizer.hpp"

Localizer::Localizer(const std::string& config_path)
{
	YAML::Node config = YAML::LoadFile(config_path.c_str());
	n_particles       = config["pf"]["n_particles"].as<int>();
	cam_pitch         = config["camera_info"]["cam_pitch"].as<double>() * PI / 180;
}

void Localizer::init(const pose6D& initial_pose)
{
	// Initialize the particle filter
	pf = new PF(n_particles, initial_pose);

	// Get the first distribution of particles
	std::vector<Particle> particles;
	(*pf).getParticles(particles);

	// Compute average pose and standard deviation of the
	// first distribution
	std::vector<pose6D> poses;
	for (size_t i = 0; i < particles.size(); i++)
		poses.push_back(particles[i].pose);
	average_pose = pose6D(poses);
}

void Localizer::process(const pose6D& odom, const std::vector<float>& bearings,
                        const std::vector<float>&             depths,
                        const std::map<int, Landmark<float>>& map)
{
	// Invocate the particle filter loop
	(*pf).process(odom, bearings, depths, map);
	// Import the resultant set of particles
	std::vector<Particle> particles;
	(*pf).getParticles(particles);

	// Compute the average pose and convert the particles pose to
	// ROS array
	std::vector<pose6D> m_poses;
	for (size_t i = 0; i < particles.size(); i++) {
		// Push back to the poses array
    pose6D m_pose = particles[i].pose;
		m_poses.push_back(m_pose);
	}
	average_pose = pose6D(m_poses);
	// Normalize average pose angles between [-pi,pi]
	average_pose.roll  = normalizeAngle(average_pose.roll);
	average_pose.pitch = cam_pitch + normalizeAngle(average_pose.pitch);
	average_pose.yaw   = normalizeAngle(average_pose.yaw);
}

pose6D Localizer::getPose() const
{
	return average_pose;
}
