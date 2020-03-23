#include "localizer.hpp"

Localizer::Localizer(const Parameters& params) : params(params) {}

void Localizer::init(const Pose<double>& initial_pose)
{
  // Initialize the particle filter
	pf = new PF(params.n_particles, initial_pose);

  // Get the first distribution of particles
	std::vector<Particle> particles;
  (*pf).getParticles(particles);

  // Compute average pose and standard deviation of the 
  // first distribution
  std::vector<Pose<double>> poses;
  for (size_t i = 0; i < particles.size(); i++)
    poses.push_back(particles[i].pose);
  average_pose = Pose<double>(poses);
}

void Localizer::process(const Pose<double>&                    odom,
                        const std::vector<double>&             bearings,
                        const std::vector<double>&             depths,
                        const std::map<int, Landmark<double>>& map)
{
  // Clear poses array
  poses.poses.clear();

	// Invocate the particle filter loop
	(*pf).process(odom, bearings, depths, map);
	// Import the resultant set of particles
	std::vector<Particle> particles;
  (*pf).getParticles(particles);

	// Compute the average pose and convert the particles pose to
	// ROS array
  std::vector<Pose<double>> m_poses;
	for (size_t i = 0; i < particles.size(); i++) {
		// Create quaternion representing the particle rotation
		tf::Quaternion q;
		q.setRPY(particles[i].pose.roll, particles[i].pose.pitch,
		         particles[i].pose.yaw);
		q.normalize();
		// Create a ROS pose representing the particle pose
		geometry_msgs::Pose pose;
		pose.position.x    = particles[i].pose.pos.x;
		pose.position.y    = particles[i].pose.pos.y;
		pose.position.z    = particles[i].pose.pos.z;
		pose.orientation.x = q.x();
		pose.orientation.y = q.y();
		pose.orientation.z = q.z();
		pose.orientation.w = q.w();
		// Push back to the poses array
		poses.poses.push_back(pose);
    m_poses.push_back(particles[i].pose);
	}
  average_pose = Pose<double>(m_poses);
	// Normalize average pose angles between [-pi,pi]
	average_pose.roll  = normalizeAngle(average_pose.roll);
	average_pose.pitch = 11.3 * PI / 180 + normalizeAngle(average_pose.pitch);
	average_pose.yaw   = normalizeAngle(average_pose.yaw);

	// Convert obtained average pose to ROS tf
  // Compute transformation relatively with camera axis but global pose
	tf::Quaternion q;
	q.setRPY(average_pose.roll, average_pose.pitch, average_pose.yaw);
	q.normalize();
	cam2map.setRotation(q);
	cam2map.setOrigin(
	    tf::Vector3(average_pose.pos.x, average_pose.pos.y, average_pose.pos.z));
}

tf::Transform Localizer::getTf() const
{
	return cam2map;
}

geometry_msgs::PoseArray Localizer::getPoseArray() const
{
	return poses;
}

Pose<double> Localizer::getPose() const 
{
  return average_pose;
}
