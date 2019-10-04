#include "../include/mapper/mapper.hpp"

ParticleFilter::ParticleFilter(const Parameters& params) : params(params)
{
	landm_obj = new LandmarkProcessor(params);
	prev_pose = Pose<double>(Point<double>(0, 0), 0);
}

void ParticleFilter::init()
{
	int n_particles = 1;
	weights.resize(n_particles);

	std::normal_distribution<double> dist_x(MEAN_X, STD_X);
	std::normal_distribution<double> dist_y(MEAN_Y, STD_Y);
	std::normal_distribution<double> dist_theta(MEAN_THETA, STD_THETA);

	std::default_random_engine generator;

	for (int i = 0; i < n_particles; i++) {
		Particle<double> p;
		p.id     = i;
		p.pos    = Point<double>(dist_x(generator), dist_y(generator));
		p.theta  = dist_theta(generator);
		p.weight = 1.0;

		particles.push_back(p);
	}
}

void ParticleFilter::process(const std::vector<Point<double>>& landm_pos,
                             const Pose<double>&               delta_pose)
{
	predict(landm_pos, delta_pose);
	updateWeights(delta_pose);
	resample();

	prev_pose.pos = prev_pose.pos + delta_pose.pos;
	prev_pose.theta += delta_pose.theta;
}

void ParticleFilter::predict(const std::vector<Point<double>>& landm_pos,
                             const Pose<double>&               delta_pose)
{
	(*landm_obj).updatePoses(landm_pos);
	(*landm_obj).matchLandmarks(prev_pose);

	std::normal_distribution<double> dist_x(delta_pose.pos.x, STD_X);
	std::normal_distribution<double> dist_y(delta_pose.pos.y, STD_Y);
	std::normal_distribution<double> dist_theta(delta_pose.theta, STD_THETA);

	std::default_random_engine generator;

	for (size_t i = 0; i < (*landm_obj).landmarks.size(); i++) {
		Landmark<double> l = (*landm_obj).landmarks[i];
		if (l.image_pos.size() <= 1)
			continue;

		Point<double> avg(0, 0);

		int inc =
		    (l.image_pos.size() >= params.rate) ? params.rate : l.image_pos.size() - 1;
		int v_size = l.image_pos.size() - 1;

		Line<double>  p_line = (*landm_obj).computeLine(l.image_pos[v_size - inc]);
		Point<double> c_pos  = l.image_pos[v_size];
		Pose<double>  d_pose = l.r_pose[v_size] - l.r_pose[v_size - inc];

		Line<double> tmp =
		    (*landm_obj).projectLine(c_pos, d_pose.pos, d_pose.theta);
		Point<double> X = p_line.intercept(tmp);

		/* calculate trunk average position estimation */
		/* avg = (X + (avg * j)) / (j + 1); */
    Point<double> res = X + l.r_pose[v_size - inc].pos;
		all_sols.push_back(res);
		(*landm_obj).landmarks[i].world_pos = res;

		std::cout << "INC:     " << inc << std::endl;
		std::cout << "X_PREV:  " << l.image_pos[v_size - inc];
		std::cout << "X_CURR:  " << l.image_pos[v_size - 1];
		std::cout << "DT_POSE: " << d_pose;
		std::cout << "RES:     " << X + l.r_pose[v_size - inc].pos;
	}

	std::cout << std::endl;
}

void ParticleFilter::updateWeights(const Pose<double>& delta_pose)
{
	double weight_sum = 0.0;

	for (size_t i = 0; i < particles.size(); i++) {
		Particle<double> p = particles[i];

		double weight =
		    exp(-0.5 * pow(delta_pose.pos.x - p.pos.x, 2) / pow(STD_X, 2)) *
		    exp(-0.5 * pow(delta_pose.pos.y - p.pos.y, 2) / pow(STD_Y, 2)) *
		    exp(-0.5 * pow(delta_pose.theta - p.theta, 2) / pow(STD_THETA, 2));

		weight_sum += weight;
		particles[i].weight = weight;
	}

	for (size_t i = 0; i < particles.size(); i++) {
		particles[i].weight /= weight_sum;
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample()
{
	std::default_random_engine gen;

	std::discrete_distribution<int> distribution(weights.begin(), weights.end());
	std::vector<Particle<double>>   resampled_particles;

	for (size_t i = 0; i < particles.size(); i++)
		resampled_particles.push_back(particles[distribution(gen)]);

	particles = resampled_particles;
}
