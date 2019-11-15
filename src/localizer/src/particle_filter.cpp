#include "../include/localizer/particle_filter.hpp"

ParticleFilter::ParticleFilter(const Parameters& params) : params(params) {}

void ParticleFilter::init(const std::vector<Landmark<double>>& landmarks)
{
	int n_particles = 1000;
	weights.resize(n_particles);

	std::normal_distribution<double> dist_x(0.0, 0);
	std::normal_distribution<double> dist_y(0.0, 0);
	std::normal_distribution<double> dist_theta(PI / 3, 0);

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

void ParticleFilter::process(const std::vector<Point<double>>& land_poses)
{
	predict(land_poses);
	updateWeights();
	resample();
}

void ParticleFilter::predict(const std::vector<Point<double>>& land_poses)
{
	for (size_t i = 0; i < particles.size(); i++) {
		Particle<double> p = particles[i];

		/* constraints for the region of interest */
		double min_thtol = columnToTheta(0);
		double max_thtol = columnToTheta(params.width);

		for (size_t j = 0; j < landmarks.size(); j++) {
			Landmark<double> l = landmarks[j];
			Point<double>    delta_pt =
			    Point<double>(l.world_pos.x - p.pos.x, p.pos.y + l.world_pos.y);

			/* check if landmark is inside the field of view */
			double th = atan2(delta_pt.y, delta_pt.x);
			double abs_min_thtol = min_thtol + p.theta;
			double abs_max_thtol = max_thtol + p.theta;
			if (th < abs_min_thtol || th > abs_max_thtol)
				continue;

			/* if so, project it on the image */
			int col = thetaToColumn(th);
      std::cout << l;
		}
	}
}

void ParticleFilter::updateWeights() {}

void ParticleFilter::resample() {}
