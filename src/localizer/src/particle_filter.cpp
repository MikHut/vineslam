#include "../include/localizer/particle_filter.hpp"

ParticleFilter::ParticleFilter(const Parameters& params) : params(params) {}

void ParticleFilter::init(const std::vector<Landmark<double>>& landmarks)
{
	int n_particles = 100;
	weights.resize(n_particles);

	std::normal_distribution<double> dist_x(0.0, STD_XY * params.scaler);
	std::normal_distribution<double> dist_y(0.0, STD_XY * params.scaler);
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

	(*this).landmarks = landmarks;
}

void ParticleFilter::process(const std::vector<Point<double>>& land_pos)
{
	predict(land_pos);
	updateWeights();
	resample();
}

void ParticleFilter::predict(const std::vector<Point<double>>& land_pos)
{
	/* get mean pose of all particles */
	double x_mean  = 0;
	double y_mean  = 0;
	double th_mean = 0;
	for (size_t i = 0; i < particles.size(); i++) {
		x_mean += particles[i].pos.x;
		y_mean += particles[i].pos.y;
		th_mean += particles[i].theta;

		particles[i].r_error = 0;
	}
	x_mean /= particles.size();
	y_mean /= particles.size();
	th_mean /= particles.size();

	/* update the particles mean considering the last estimation */
	std::normal_distribution<double> dist_x(x_mean, STD_XY * params.scaler);
	std::normal_distribution<double> dist_y(y_mean, STD_XY * params.scaler);
	std::normal_distribution<double> dist_theta(th_mean, STD_THETA);

	std::default_random_engine generator;

	for (size_t i = 0; i < particles.size(); i++) {
		particles[i].pos   = Point<double>(dist_x(generator), dist_y(generator));
		particles[i].theta = dist_theta(generator);

		Particle<double> p = particles[i];

		/* constraints for the region of interest */
		double min_thtol = columnToTheta(0);
		double max_thtol = columnToTheta(params.width);

		int n_corr = 0;
		for (size_t j = 0; j < landmarks.size(); j++) {
			Landmark<double> l = landmarks[j];
			Point<double>    delta_pt(l.world_pos.x - p.pos.x * params.scaler,
                             p.pos.y * params.scaler + l.world_pos.y);

			/* check if landmark is inside the field of view */
			double th            = atan2(delta_pt.y, delta_pt.x);
			double abs_min_thtol = min_thtol + p.theta;
			double abs_max_thtol = max_thtol + p.theta;
			if (th < abs_min_thtol || th > abs_max_thtol)
				continue;

			/* if so, project it on the image */
			int col = thetaToColumn(th);

			/* search for correspondences and calculate the reprojection error */
			for (size_t k = 0; k < land_pos.size(); k++) {
				Point<double> l_detected = land_pos[k];
				double        r_error    = pow(l_detected.x - col, 2);
				if (r_error < 10) {
					n_corr++;
					particles[i].r_error += r_error;
				}
			}
		}
		particles[i].r_error = (n_corr == 0) ? INF : particles[i].r_error / n_corr;
		if (n_corr > 0) {
			std::cout << n_corr << std::endl;
			std::cout << particles[i] << std::endl;
		}
	}
}

void ParticleFilter::updateWeights()
{
	double weight_sum = 0.0;

	for (size_t i = 0; i < particles.size(); i++) {
		Particle<double> p = particles[i];
		double weight = (p.r_error == INF) ? 0 : exp(-0.5 * particles[i].r_error);

		weight_sum += weight;
		particles[i].weight = weight;
	}

	for (size_t i = 0; i < particles.size(); i++) {
		if (weight_sum > 0.0) {
			particles[i].weight /= weight_sum;
			weights[i] = particles[i].weight;
		}
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
