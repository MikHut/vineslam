#include "pf.hpp"

PF::PF(const std::string& config_path, const int& n_particles,
       const pose6D& initial_pose)
    : config_path(config_path)
{
	// Read input parameters
	YAML::Node config = YAML::LoadFile(config_path.c_str());
	cam_pitch   = config["camera_info"]["cam_pitch"].as<double>() * PI / 180;
	alpha_trans = config["pf"]["alpha_trans"].as<float>();
	alpha_rot   = config["pf"]["alpha_rot"].as<float>();

	// Declare mean and std of each gaussian
	float std_xy  = 0.5;             // alpha a meter of initial uncertainty
	float std_rpy = 10.0 * PI / 180; // ten degrees of initial uncertainty

	// Initialize normal distributions
	std::default_random_engine      generator;
	std::normal_distribution<float> gauss_x(initial_pose.x, std_xy);
	std::normal_distribution<float> gauss_y(initial_pose.y, std_xy);
	std::normal_distribution<float> gauss_roll(0.0, 0.0);
	std::normal_distribution<float> gauss_pitch(cam_pitch, std_rpy);
	std::normal_distribution<float> gauss_yaw(initial_pose.yaw, std_rpy);

	// Resize particles array
	particles.resize(n_particles);

	// Initialize all particles
	for (int i = 0; i < n_particles; i++) {
		// Calculate the initial pose for each particle considering
		// - the input initial pose
		// - a sample distribution to spread the particles
		pose6D pose(gauss_x(generator), gauss_y(generator), 0., 0.,
		            gauss_pitch(generator), gauss_yaw(generator));
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
	// Resample all particles only if the filter receives new information
	if (bearings.size() > 0)
		resample();
}

void PF::predict(const pose6D& odom)
{
	// Compute the relative pose given by the odometry motion model
	float dt_trans = sqrt(pow(odom.x - p_odom.x, 2) + pow(odom.y - p_odom.y, 2));
	float dt_rot_a =
	    normalizeAngle(atan2(odom.y - p_odom.y, odom.x - p_odom.x) - p_odom.yaw);
	float dt_rot_b = normalizeAngle(odom.yaw - p_odom.yaw - dt_rot_a);

	// Standard deviations of the odometry motion model
	float std_trans = dt_trans * alpha_trans;
	float std_rot   = (dt_rot_a + dt_rot_b) * alpha_rot;

	// Standard deviation of the non-observable states (roll, pitch)
	float std_rp = 1.5 * PI / 180;

	// Declare normal Gaussian distributions to innovate the particles
	std::default_random_engine      generator;
	std::normal_distribution<float> gauss_trans(0.0, std_trans);
	std::normal_distribution<float> gauss_rot(0.0, std_rot);
	std::normal_distribution<float> gauss_rp(0.0, 0.0);

	// Apply the motion model to all particles
	for (size_t i = 0; i < particles.size(); i++) {
		// Sample the normal distribution functions
		float s_rot_a = dt_rot_a + gauss_rot(generator);
		float s_rot_b = dt_rot_b + gauss_rot(generator);
		float s_trans = dt_trans + gauss_trans(generator);
		float s_r     = gauss_rp(generator);
		float s_p     = gauss_rp(generator);

		// Compute the relative pose considering the samples
		float  p_yaw = particles[i].pose.yaw;
		pose6D dt_pose;
		dt_pose.x     = s_trans * cos(normalizeAngle(p_yaw + s_rot_a));
		dt_pose.y     = s_trans * sin(normalizeAngle(p_yaw + s_rot_a));
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
		// Convert particle i pose to Rotation matrix
		std::vector<float> Rot;
		particles[i].pose.toRotMatrix(Rot);

		for (size_t j = 0; j < bearings.size(); j++) {
			// Calculate the estimation of the landmark on particles
			// referential frame
			float   th = bearings[j];
			point3D X_local(depths[j] * cos(th), depths[j] * sin(th), 0.);
			// Convert landmark to map's referential frame considering the
			// particle pose
			point3D X;
			X.x = X_local.x * Rot[0] + X_local.y * Rot[1] + X_local.z * Rot[2] +
			      particles[i].pose.x;
			X.y = X_local.x * Rot[3] + X_local.y * Rot[4] + X_local.z * Rot[5] +
			      particles[i].pose.y;
			X.z = 0.;

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
		int   j      = 1;

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

void PF::setSensorData(float*                               raw_depths,
                       const vision_msgs::Detection2DArray& dets)
{
	(*this).raw_depths  = raw_depths;
	(*this).dets        = dets;
}

