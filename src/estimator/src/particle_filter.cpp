#include "../include/particle_filter.hpp"

ParticleFilter::ParticleFilter(const Parameters& params) : params(params)
{
	landm_obj = new LandmarkProcessor(params);
}

void ParticleFilter::init()
{
	int n_particles = 1000;
	weights.resize(n_particles);

	std::normal_distribution<double> dist_x(5.0, STD_XY);
	std::normal_distribution<double> dist_y(0.0, STD_XY);
	std::normal_distribution<double> dist_theta(0.0, STD_THETA);

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
#ifdef VISUALIZE
	(*landm_obj).p_map = cv::Mat(params.resolution, params.resolution, CV_8UC1,
	                             cv::Scalar(255, 255, 255));
#endif

	predict(landm_pos);
	updateWeights(delta_pose);
	resample();
}

void ParticleFilter::predict(const std::vector<Point<double>>& landm_pos)
{
	(*landm_obj).updatePoses(landm_pos);
	(*landm_obj).matchLandmarks();

	for (size_t i = 0; i < (*landm_obj).matches.size(); i++) {
		Match<double> m = (*landm_obj).matches[i];
		(*landm_obj).plotGrid(m.p_line, i * 30);
		for (size_t j = 0; j < particles.size(); j++) {
			Particle<double> p = particles[j];

			Line<double>  pl  = m.p_line;
			Line<double>  cl  = m.c_line;
			Line<double>  tmp = (*landm_obj).projectLine(m, p.pos, p.theta);
			Point<double> X   = pl.intercept(tmp);

			(*landm_obj).plotPMap(X, i * 30);

			(*landm_obj).matches[i].l.pos.push_back(X);
		}
	}
}

void ParticleFilter::updateWeights(const Pose<double>& delta_pose)
{
	double weight_sum = 0.0;

	for (size_t i = 0; i < particles.size(); i++) {
		Particle<double> p = particles[i];

		double weight =
		    exp(-0.5 * pow(delta_pose.pos.x - p.pos.x, 2) / pow(STD_XY, 2)) *
		    exp(-0.5 * pow(delta_pose.pos.y - p.pos.y, 2) / pow(STD_XY, 2)) *
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
