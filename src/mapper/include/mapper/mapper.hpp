#pragma once

#include "landmark_processor.hpp"
#include "pf.hpp"
#include "utils.hpp"
#include <iostream>
#include <math.h>

class Estimator
{
public:
	Estimator(const Parameters& params, LandmarkProcessor* lprocessor);
	void process(const std::vector<Pose<double>>& robot_pose,
	             const std::vector<int>&          index);

private:
	double columnToTheta(const int& col)
	{
		return (-params.h_fov / params.width) * (params.width / 2 - col);
	}

	Parameters         params;
	LandmarkProcessor* lprocessor;
};
