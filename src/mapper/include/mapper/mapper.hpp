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
	void          pfPrediction(const std::vector<Pose<double>>& robot_poses,
	                           const std::vector<int>&          index);
	void          kfPrediction(const std::vector<Pose<double>>& robot_poses,
	                           const std::vector<int>&          index);
	Point<double> processObsv(const Landmark<double>& l, const int& it,
	                          const Pose<double>& delta_p);
	void          control();

	double columnToTheta(const int& col)
	{
		return (-params.h_fov / params.width) * (params.width / 2 - col);
	}

	Parameters         params;
	LandmarkProcessor* lprocessor;

	Line<double> vine_right;
	Line<double> vine_left;
};
