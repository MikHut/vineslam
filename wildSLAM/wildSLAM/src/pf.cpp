#include "pf.hpp"

PF::PF(const int& n_particles, const Pose<double>& initial_pose)
{
	// Declare mean and std of each gaussian
	double std_xy  = 0.5;            // alpha a meter of initial uncertainty
	double std_rpy = 10.0 * PI / 180; // ten degrees of initial uncertainty

	// Initialize normal distributions
	std::default_random_engine       generator;
	std::normal_distribution<double> gauss_x(initial_pose.pos.x, std_xy);
	std::normal_distribution<double> gauss_y(initial_pose.pos.y, std_xy);
	std::normal_distribution<double> gauss_rp(0.0, std_rpy);
	std::normal_distribution<double> gauss_yaw(initial_pose.yaw, std_rpy);

	// Resize particles array
	particles.resize(n_particles);

	// Initialize all particles
	for (int i = 0; i < n_particles; i++) {
		// Calculate the initial pose for each particle considering
		// - the input initial pose
		// - a sample distribution to spread the particles
		Pose<double> pose(gauss_x(generator), gauss_y(generator), 0,
		                  gauss_rp(generator), gauss_rp(generator),
		                  gauss_yaw(generator));
		// Compute initial weight of each particle
		double weight = 1 / (double)n_particles;
		// Insert the particle into the particles array
		particles[i] = Particle(i, pose, weight);
	}

	// Initialize the previous pose
	p_odom = initial_pose;
}

void PF::process(const Pose<double>& odom, const std::vector<double>& bearings,
                 const std::vector<double>&             depths,
                 const std::map<int, Landmark<double>>& map)
{
	// Invocate prediction step to inovate the particles
	predict(odom);
	// Invocate the correction step to compute the weights
	correct(bearings, depths, map);
	// Resample all particles
	resample();
}

void PF::predict(const Pose<double>& odom)
{
	// Compute the relative pose given by the odometry motion model
	double dt_trans = sqrt(pow(odom.pos.x - p_odom.pos.x, 2) +
	                       pow(odom.pos.y - p_odom.pos.y, 2));
	double dt_rot_a = normalizeAngle(
	    atan2(odom.pos.y - p_odom.pos.y, odom.pos.x - p_odom.pos.x) - p_odom.yaw);
	double dt_rot_b = normalizeAngle(odom.yaw - p_odom.yaw - dt_rot_a);

	// Define alphas to calculate the standard deviations of the samples
	double alpha_1 = 0.5;
	double alpha_2 = 0.5;
	double alpha_3 = 0.5;
	double alpha_4 = 0.5;

	// Standard deviations of the odometry motion model
	double std_rot_a =
	    alpha_1 * std::fabs(dt_rot_a) + alpha_2 * std::fabs(dt_trans);
	double std_trans =
	    alpha_3 * std::fabs(dt_trans) + alpha_4 * std::fabs(dt_rot_a * dt_rot_b);
	double std_rot_b =
	    alpha_1 * std::fabs(dt_rot_b) + alpha_2 * std::fabs(dt_trans);

	// Standard deviation of the non-observable states (roll, pitch)
	double std_rp = 1.5 * PI / 180;

	// Declare normal Gaussian distributions to innovate the particles
	std::default_random_engine       generator;
	std::normal_distribution<double> gauss_trans(0.0, std_trans);
	std::normal_distribution<double> gauss_rot_a(0.0, std_rot_a);
	std::normal_distribution<double> gauss_rot_b(0.0, std_rot_b);
	std::normal_distribution<double> gauss_rp(0.0, std_rp);

	// Apply the motion model to all particles
	for (size_t i = 0; i < particles.size(); i++) {
		// Sample the normal distribution functions
		double s_rot_a = dt_rot_a + gauss_rot_a(generator);
		double s_rot_b = dt_rot_b + gauss_rot_b(generator);
		double s_trans = dt_trans + gauss_trans(generator);
		double s_r     = gauss_rp(generator);
		double s_p     = gauss_rp(generator);

		// Compute the relative pose considering the samples
		double       p_yaw = particles[i].pose.yaw;
		Pose<double> dt_pose;
		dt_pose.pos.x = dt_trans * cos(normalizeAngle(p_yaw + s_rot_a));
		dt_pose.pos.y = dt_trans * sin(normalizeAngle(p_yaw + s_rot_a));
		dt_pose.pos.z = 0.0;
		dt_pose.roll  = s_r;
		dt_pose.pitch = s_p;
		dt_pose.yaw   = normalizeAngle(s_rot_a + s_rot_b);

		// Innovate particles using the odometry motion model
		particles[i].pose.pos.x += dt_pose.pos.x;
		particles[i].pose.pos.y += dt_pose.pos.y;
		particles[i].pose.pos.z += dt_pose.pos.z;
		particles[i].pose.roll += dt_pose.roll;
		particles[i].pose.pitch += dt_pose.pitch;
		particles[i].pose.yaw += dt_pose.yaw;
	}

	p_odom = odom;
}

void PF::correct(const std::vector<double>&             bearings,
                 const std::vector<double>&             depths,
                 const std::map<int, Landmark<double>>& map)
{
	double weights_sum = 0.0;

	// Loop over all particles
	for (size_t i = 0; i < particles.size(); i++) {
		// Calculation of a local map for each particle
		double error_sum = 0.0;
		for (size_t j = 0; j < bearings.size(); j++) {
		//for (size_t j = 0; j < 0; j++) {
			// Calculate the estimation of the landmark
			double        th = bearings[j] + particles[i].pose.yaw;
			Point<double> X(particles[i].pose.pos.x + depths[j] * cos(th),
			                particles[i].pose.pos.y + depths[j] * sin(th));

			// Loop over landmarks on global map in local map
			// to get the best correspondence
			double best_correspondence = 1e6;
			for (auto m_map : map) {
				double dist_min = X.euc_dist(m_map.second.pos);

				if (dist_min < best_correspondence)
					best_correspondence = dist_min;
			}
			error_sum += pow(best_correspondence, 2);
		}

		// Save the particle i weight
		particles[i].w = (1 / sqrt(2 * PI)) * exp(-pow(error_sum, 2) / 2);
		//particles[i].w = (error_sum > 0) ? (1 / error_sum) : 0.0;
		weights_sum += particles[i].w;
	}
	// Normalize the particles weights to [0,1]
	if (weights_sum > 0) {
		for (size_t i = 0; i < particles.size(); i++)
			particles[i].w /= weights_sum;
	}
	else {
		for (size_t i = 0; i < particles.size(); i++)
			particles[i].w = 1 / (double)particles.size();
	}
}

void PF::resample()
{
	const int M = particles.size();

	// Construct array with all particles weights
	std::vector<double> w;
	for (int i = 0; i < M; i++)
		w.push_back(particles[i].w);

	// Cumulative sum of weights
	std::vector<double> Q(M);
	Q[0] = w[0];
	for (int i = 1; i < M; i++)
		Q[i] = Q[i - 1] + w[i];

	// Perform multinomial resampling
	int              i = 0;
	std::vector<int> index(M);
	while (i < M) {
		double sample = ((double)std::rand() / (RAND_MAX));
		int    j      = 1;

		while (Q[j] < sample)
			j++;

		index[i] = j;
		i++;
	}

	// Update set of particles with indexes resultant from the
	// resampling procedure
	for (i = 0; i < M; i++) {
		particles[i].pose = particles[index[i]].pose;
		particles[i].w    = particles[index[i]].w;
	}
}

void PF::getParticles(std::vector<Particle>& in)
{
	in.resize(particles.size());
	for (size_t i = 0; i < particles.size(); i++)
		in[i] = particles[i];
}
