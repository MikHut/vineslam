#include "pf.hpp"

PF::PF(const int& n_particles, const Pose<double>& initial_pose)
{
	// Declare mean and std of each gaussian
	double mean    = 0;
	double std_xyz = 0.5;
	double std_rpy = 5.0 * PI / 180;

	// Initialize normal distributions
	std::default_random_engine       generator;
	std::normal_distribution<double> gauss_xy(mean, std_xyz);
	std::normal_distribution<double> gauss_rpy(mean, std_rpy);

	// Resize particles array
	particles.resize(n_particles);

	// Initialize all particles
	for (int i = 0; i < n_particles; i++) {
		// Calculate the initial pose for each particle considering
		// - the input initial pose
		// - a sample distribution to spread the particles
		Pose<double> sample(gauss_xy(generator), gauss_xy(generator), 0,
		                    gauss_rpy(generator), gauss_rpy(generator),
		                    gauss_rpy(generator));
		Pose<double> pose = sample + initial_pose;
		// Compute initial weight of each particle
		double weight = 1 / (double)n_particles;
		// Insert the particle into the particles array
		particles[i] = Particle(i, pose, weight);
	}

	// Set previous pose to zero
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
	// Compute the relative pose given by odometry
	double delta_trans = sqrt(pow(odom.pos.x - p_odom.pos.x, 2) +
	                          pow(odom.pos.y - p_odom.pos.y, 2));
	double delta_rot_a = normalizeAngle(
	    atan2(odom.pos.y - p_odom.pos.y, odom.pos.x - p_odom.pos.x) - p_odom.yaw);
	double delta_rot_b = normalizeAngle(odom.yaw - p_odom.yaw - delta_rot_a);

	Pose<double> delta_pose(delta_trans * cos(p_odom.yaw + delta_rot_a),
	                        delta_trans * sin(p_odom.yaw + delta_rot_a), 0, 0, 0,
	                        delta_rot_a + delta_rot_b);

	// Declare normal Gaussian distributions to innovate the particles
	double mean    = 0.0;
	double std_xyz = delta_trans * 0.5;
	double std_rp  = 1.5 * PI / 180;
	double std_yaw = std::fabs(normalizeAngle(delta_rot_a + delta_rot_b)) * 0.5;
	std::default_random_engine       generator;
	std::normal_distribution<double> gauss_xyz(mean, std_xyz);
	std::normal_distribution<double> gauss_rp(mean, std_rp);
	std::normal_distribution<double> gauss_yaw(mean, std_yaw);

	// Apply the motion model to all particles
	for (size_t i = 0; i < particles.size(); i++) {
		// Compute the sample pose applying gaussian noise to the
		// motion model
		Pose<double> sample(gauss_xyz(generator), gauss_xyz(generator), 0,
		                    gauss_rp(generator), gauss_rp(generator),
		                    gauss_yaw(generator));
		// Compute the new particle 6-DOF pose
		particles[i].pose = particles[i].pose + delta_pose + sample;
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
		particles[i].w = (error_sum > 0) ? (1 / error_sum) : 0.0;
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
