#include "../include/particle_filter.hpp"

ParticleFilter::ParticleFilter(const Parameters& params) : params(params)
{
	landm_obj = new LandmarkProcessor(params);
}

void ParticleFilter::init()
{
	int    id     = 0;
	double x      = -MAX_DXY;
	double y      = -MAX_DXY;
	double theta  = -MAX_DTHETA / 2;
	double weight = 0.0;

	while (x <= MAX_DXY) {
		while (y <= MAX_DXY) {
			while (theta <= MAX_DTHETA / 2) {
				particles.push_back(
				    Particle<double>(id, Point<double>(x, y), theta, weight));
				theta += RAD;
				id++;
			}
			theta = -MAX_DTHETA / 2;
			y += 0.1;
		}
		y = -MAX_DXY;
		x += 0.1;
	}
}

void ParticleFilter::process(const std::vector<Point<double>>& poses)
{
	(*landm_obj).process(poses, particles);
#ifdef VISUALIZE
	plotParticles();
#endif
	updateWeights();
}

void ParticleFilter::updateWeights()
{
	for (size_t i = 0; i < particles.size(); i++)
		particles[i].weight = 0;
}

void ParticleFilter::plotParticles()
{
	box = cv::Mat(2 * MAX_DXY * 100 + 1, 2 * MAX_DXY * 100 + 1, CV_8UC1,
	              cv::Scalar(0, 0, 0));

	for (size_t i = 0; i < particles.size(); i++) {
		Point<double> pt((particles[i].pos.x + MAX_DXY) * 100,
		                 box.rows - (particles[i].pos.y + MAX_DXY) * 100);
		if (particles[i].weight > 0) {
			cv::circle(box, cv::Point2f(pt.y, pt.x), 1,
			           cv::Scalar(particles[i].weight * 40, 0, 0), 0.5);
		}
	}
}
