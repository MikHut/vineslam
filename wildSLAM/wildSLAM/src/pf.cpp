#include "pf.hpp"

PF::PF(const int& n_particles, const pose6D& initial_pose)
{
	// Declare mean and std of each gaussian
	float std_xy  = 0.5;             // alpha a meter of initial uncertainty
	float std_rpy = 10.0 * PI / 180; // ten degrees of initial uncertainty

	// Initialize normal distributions
	std::default_random_engine       generator;
	std::normal_distribution<float> gauss_x(initial_pose.x, std_xy);
	std::normal_distribution<float> gauss_y(initial_pose.y, std_xy);
	std::normal_distribution<float> gauss_rp(0.0, 0.0);
	std::normal_distribution<float> gauss_yaw(initial_pose.yaw, std_rpy);

	// Resize particles array
	particles.resize(n_particles);

	// Initialize all particles
	for (int i = 0; i < n_particles; i++) {
		// Calculate the initial pose for each particle considering
		// - the input initial pose
		// - a sample distribution to spread the particles
		pose6D pose(gauss_x(generator), gauss_y(generator), 0, gauss_rp(generator),
		            gauss_rp(generator), gauss_yaw(generator));
		// Compute initial weight of each particle
		float weight = 1 / (float)n_particles;
		// Insert the particle into the particles array
		particles[i] = Particle(i, pose, weight);
	}

	// Initialize the previous pose
	p_odom = initial_pose;
}

void PF::process(const pose6D& odom, const std::vector<float>& bearings,
                 const std::vector<float>&             depths,
                 const std::map<int, Landmark<float>>& map)
{
	// Invocate prediction step to inovate the particles
	predict(odom);
	// Invocate the correction step to compute the weights
	correct(bearings, depths, map);
	// Resample all particles
	resample();
}

void PF::predict(const pose6D& odom)
{
	// Compute the relative pose given by the odometry motion model
	float dt_trans = sqrt(pow(odom.x - p_odom.x, 2) + pow(odom.y - p_odom.y, 2));
	float dt_rot_a =
	    normalizeAngle(atan2(odom.y - p_odom.y, odom.x - p_odom.x) - p_odom.yaw);
	float dt_rot_b = normalizeAngle(odom.yaw - p_odom.yaw - dt_rot_a);

	// Define alphas to calculate the standard deviations of the samples
	float alpha_1 = 0.5;
	float alpha_2 = 0.5;
	float alpha_3 = 0.5;
	float alpha_4 = 0.5;

	// Standard deviations of the odometry motion model
	float std_rot_a =
	    alpha_1 * std::fabs(dt_rot_a) + alpha_2 * std::fabs(dt_trans);
	float std_trans =
	    alpha_3 * std::fabs(dt_trans) + alpha_4 * std::fabs(dt_rot_a * dt_rot_b);
	float std_rot_b =
	    alpha_1 * std::fabs(dt_rot_b) + alpha_2 * std::fabs(dt_trans);

	// Standard deviation of the non-observable states (roll, pitch)
	float std_rp = 1.5 * PI / 180;

	// Declare normal Gaussian distributions to innovate the particles
	std::default_random_engine       generator;
	std::normal_distribution<float> gauss_trans(0.0, std_trans);
	std::normal_distribution<float> gauss_rot_a(0.0, std_rot_a);
	std::normal_distribution<float> gauss_rot_b(0.0, std_rot_b);
	std::normal_distribution<float> gauss_rp(0.0, 0.0);

	// Apply the motion model to all particles
	for (size_t i = 0; i < particles.size(); i++) {
		// Sample the normal distribution functions
		float s_rot_a = dt_rot_a + gauss_rot_a(generator);
		float s_rot_b = dt_rot_b + gauss_rot_b(generator);
		float s_trans = dt_trans + gauss_trans(generator);
		float s_r     = gauss_rp(generator);
		float s_p     = gauss_rp(generator);

		// Compute the relative pose considering the samples
		float p_yaw = particles[i].pose.yaw;
		pose6D dt_pose;
		dt_pose.x     = dt_trans * cos(normalizeAngle(p_yaw + s_rot_a));
		dt_pose.y     = dt_trans * sin(normalizeAngle(p_yaw + s_rot_a));
		dt_pose.z     = 0.0;
		dt_pose.roll  = 0.0;
		dt_pose.pitch = 0.0;
		dt_pose.yaw   = normalizeAngle(s_rot_a + s_rot_b);

		// Innovate particles using the odometry motion model
		particles[i].pose.x += dt_pose.x;
		particles[i].pose.y += dt_pose.y;
		particles[i].pose.z += dt_pose.z;
		particles[i].pose.roll += dt_pose.roll;
		particles[i].pose.pitch += dt_pose.pitch;
		particles[i].pose.yaw += dt_pose.yaw;
	}

	p_odom = odom;
}

void PF::correct(const std::vector<float>&             bearings,
                 const std::vector<float>&             depths,
                 const std::map<int, Landmark<float>>& map)
{
	float weights_sum = 0.0;

	// Loop over all particles
	for (size_t i = 0; i < particles.size(); i++) {
		// Calculation of a local map for each particle
		float error_sum = 0.0;
		for (size_t j = 0; j < bearings.size(); j++) {
			// for (size_t j = 0; j < 0; j++) {
			// Calculate the estimation of the landmark
			float  th = bearings[j] + particles[i].pose.yaw;
			point3D X(particles[i].pose.x + depths[j] * cos(th),
			          particles[i].pose.y + depths[j] * sin(th), 0.);

			// Loop over landmarks on global map in local map
			// to get the best correspondence
			float best_correspondence = 1e6;
			for (auto m_map : map) {
				float dist_min = X.distanceXY(m_map.second.pos);

				if (dist_min < best_correspondence)
					best_correspondence = dist_min;
			}
			error_sum += pow(best_correspondence, 2);
		}

		// Save the particle i weight
		particles[i].w = (1 / sqrt(2 * PI)) * exp(-pow(error_sum, 2) / 2);
		// particles[i].w = (error_sum > 0) ? (1 / error_sum) : 0.0;
		weights_sum += particles[i].w;
	}
	// Normalize the particles weights to [0,1]
	if (weights_sum > 0) {
		for (size_t i = 0; i < particles.size(); i++)
			particles[i].w /= weights_sum;
	}
	else {
		for (size_t i = 0; i < particles.size(); i++)
			particles[i].w = 1 / (float)particles.size();
	}
}

void PF::resample()
{
	const int M = particles.size();

	// Construct array with all particles weights
	std::vector<float> w;
	for (int i = 0; i < M; i++)
		w.push_back(particles[i].w);

	// Cumulative sum of weights
	std::vector<float> Q(M);
	Q[0] = w[0];
	for (int i = 1; i < M; i++)
		Q[i] = Q[i - 1] + w[i];

	// Perform multinomial resampling
	int              i = 0;
	std::vector<int> index(M);
	while (i < M) {
		float sample = ((float)std::rand() / (RAND_MAX));
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
