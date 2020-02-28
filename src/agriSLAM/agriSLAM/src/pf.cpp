#include "pf.hpp"

PF::PF(const int& n_particles, const Pose<double>& initial_pose)
{
	// Initialize all particles
	for (int i = 0; i < n_particles; i++) {
		// Calculate the initial pose for each particle considering
		// - the input initial pose
		// - a sample distribution to spread the particles
		Pose<double> sample((rand() % 2) - 0.5, (rand() % 2) - 0.5, 0,
		                    ((rand() % 2) - 0.5) / 180 * PI * 10,
		                    ((rand() % 2) - 0.5) / 180 * PI * 10,
		                    ((rand() % 2) - 0.5) / 180 * PI * 5);
		Pose<double> pose = sample + initial_pose;
		// Compute initial weight of each particle
		double weight = 1 / n_particles;
		// Insert the particle into the particles array
		Particle p(i, pose, weight);
		particles.push_back(p);
	}
}

void PF::process(const Pose<double>& odom, const std::vector<double>& bearings,
                 const std::vector<double>&             depths,
                 const std::map<int, Landmark<double>>& map)
{
	// Invocate prediction step to inovate the particles
	predict(odom);
	// Invocate the correction step to compute the weights
	correct(odom, bearings, depths, map);
	// Resample all particles
	resample();
}

void PF::predict(const Pose<double>& odom)
{
	// Compute the relative pose given by odometry
	double delta_trans = sqrt(pow(odom.pos.x - p_odom.pos.x, 2) +
	                          pow(odom.pos.y - p_odom.pos.y, 2));
	double delta_rot_a =
	    atan2(odom.pos.y - p_odom.pos.y, odom.pos.x - p_odom.pos.x) - p_odom.yaw;
	double delta_rot_b = odom.yaw - p_odom.yaw - delta_rot_a;

	Pose<double> delta_pose(delta_trans * cos(p_odom.yaw + delta_rot_a),
	                        delta_trans * sin(p_odom.yaw + delta_rot_a), 0,
	                        delta_rot_a + delta_rot_b, 0, 0);


	// Apply the motion model to all particles
	for (size_t i = 0; i < particles.size(); i++) {
		// Compute the sample pose applying gaussian noise to the
		// motion model
		Pose<double> sample((rand() % 2) - 0.5, (rand() % 2) - 0.5, 0,
		                    ((rand() % 2) - 0.5) / 180 * PI,
		                    ((rand() % 2) - 0.5) / 180 * PI,
		                    ((rand() % 2) - 0.5) / 180 * PI);
		// Compute the new particle 6-DOF pose
		Pose<double> new_pose = particles[i].pose + delta_pose + sample;
	}

	p_odom = odom;
}

void PF::correct(const Pose<double>& odom, const std::vector<double>& bearings,
                 const std::vector<double>&             depths,
                 const std::map<int, Landmark<double>>& map)
{
	// Loop over all particles
	for (size_t i = 0; i < particles.size(); i++) {
		// Calculation of a local map for each particle
    double error_sum = 0.0;
		for (size_t j = 0; j < bearings.size(); j++) {
			// Calculate
			// - the estimation of the landmark
			// - the observation covariance of the landmark
			double        th = bearings[j] + odom.yaw;
			Point<double> X(odom.pos.x + depths[j] * cos(th),
			                odom.pos.y + depths[j] * sin(th));
			// Loop over landmarks on global map to find correspondences
			// in local map
			for (auto m_map : map) {
				double dist_x = X.x - m_map.second.pos.x;
				double dist_y = X.y - m_map.second.pos.y;
				double std_x  = 3 * m_map.second.stdev.std_x;
				double std_y  = 3 * m_map.second.stdev.std_y;

				// Check if a correspondence was found
				if (std::fabs(dist_x) < std_x && std::fabs(dist_y) < std_y) {
          // If so, sum the euclidean distance and break the loop
          error_sum += X.euc_dist(m_map.second.pos);
          break;
				}
			}
		}
    particles[i].w = (error_sum > 0) ? (1 / error_sum) : 0.0;
	}
}

void PF::resample()
{
	// TODO
}
