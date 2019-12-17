#pragma once

#include "kf.hpp"
#include "landmark_processor.hpp"
#include "utils.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <math.h>
#include <numeric>

class Estimator
{
public:
	Estimator(const Parameters& params, LandmarkProcessor* lprocessor);
	void process(const std::vector<Pose<double>>& robot_poses,
	             const std::vector<int>&          index);
	void singleDraw(const std::vector<Pose<double>>& robot_poses, const int& id);

	std::vector<Point<double>>    all_sols;
	std::vector<Landmark<double>> m_landmarks;

	std::map<int, KF> kf;

private:
	void filterXYTheta(const std::vector<Pose<double>> robot_poses,
	                   std::vector<Pose<double>>&      filtered_poses);

	void predict(const std::vector<Pose<double>>& robot_poses,
	             const std::vector<int>&          index);

	Point<double> processObsv(const Landmark<double>& l, const int& it,
	                          const Pose<double>& delta_p);
	void          initLandmark(const std::vector<Pose<double>>& robot_poses,
	                           const int&                       index);

	double columnToTheta(const int& col)
	{
		return (-params.h_fov / params.width) * (params.width / 2 - col);
	}

	Parameters         params;
	LandmarkProcessor* lprocessor;
	Pose<double>       prev_pose;
};
