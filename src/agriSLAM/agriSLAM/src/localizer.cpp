#include "localizer.hpp"

Localizer::Localizer(const Parameters& params) : params(params) {}

void Localizer::init(const Pose<double>& initial_pose)
{
	pf = new PF(params.n_particles, initial_pose);
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
	average_pose = Pose<double>(0, 0, 0, 0, 0, 0);
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

		// Calculate average pose
		average_pose.pos.x =
		    ((average_pose.pos.x * i) + particles[i].pose.pos.x) / (i + 1);
		average_pose.pos.y =
		    ((average_pose.pos.y * i) + particles[i].pose.pos.y) / (i + 1);
		average_pose.pos.z =
		    ((average_pose.pos.z * i) + particles[i].pose.pos.z) / (i + 1);
		average_pose.pos.z =
		    ((average_pose.roll * i) + particles[i].pose.roll) / (i + 1);
		average_pose.pitch =
		    ((average_pose.pitch * i) + particles[i].pose.pitch) / (i + 1);
		average_pose.yaw =
		    ((average_pose.yaw * i) + particles[i].pose.yaw) / (i + 1);
	}
	// Normalize average pose angles between [-pi,pi]
	average_pose.roll  = normalizeAngle(average_pose.roll);
	average_pose.pitch = normalizeAngle(average_pose.pitch);
	average_pose.yaw   = normalizeAngle(average_pose.yaw);

	// Convert obtained average pose to ROS tf
	tf::Quaternion q;
	q.setRPY(average_pose.roll, average_pose.pitch, average_pose.yaw);
	q.normalize();
	cam2world.setRotation(q);
	cam2world.setOrigin(
	    tf::Vector3(average_pose.pos.x, average_pose.pos.y, average_pose.pos.z));
}

tf::Transform Localizer::getTf() const
{
	return cam2world;
}

geometry_msgs::PoseArray Localizer::getPoseArray() const
{
	return poses;
}

Pose<double> Localizer::getPose() const 
{
  return average_pose;
}
