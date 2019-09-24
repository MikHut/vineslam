#pragma once

#include "landmark_processor.hpp"
#include "utils.hpp"
#include <iostream>
#include <math.h>

class ParticleFilter
{
public:
	ParticleFilter(const Parameters& params);
	void init();
	void process(const std::vector<Point<double>>& poses,
	             const Pose<double>&               delta_pose);

	LandmarkProcessor*            landm_obj;
	std::vector<Particle<double>> particles;
	std::vector<Landmark<double>> landmarks;
	cv::Mat                       box;

private:
	void predict(const std::vector<Point<double>>& landm_pos,
	             const Pose<double>&               delta_pose);
	void updateWeights(const Pose<double>& delta_pose);
	void resample();

	Parameters                    params;
	Pose<double>                  prev_pose;
	std::vector<double>           weights;
};
