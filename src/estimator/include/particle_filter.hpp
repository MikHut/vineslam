#pragma once

#include "landmark_processor.hpp"
#include "utils.hpp"
#include <iostream>

class ParticleFilter
{
public:
	ParticleFilter(const Parameters& params);
	void init();
	void process(const std::vector<Point<double>>& poses);
	void plotParticles();

	LandmarkProcessor*            landm_obj;
	std::vector<Particle<double>> particles;
	cv::Mat                       box;

private:
	void updateWeights();

	Parameters params;
};
