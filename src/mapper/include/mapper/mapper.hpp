#pragma once

#include "../../utils/utils.hpp"
#include "landmark_processor.hpp"
#include <iostream>
#include <math.h>
#include <numeric>

class Estimator
{
public:
	Estimator(const Parameters& params, LandmarkProcessor* lprocessor);
	void process(const std::vector<Pose<double>>& robot_poses);
	void singleDraw(const std::vector<Pose<double>>& robot_poses, const int& id);

	std::vector<Point<double>>    all_sols;
	std::vector<Landmark<double>> m_landmarks;

	cv::Mat map;
	cv::Mat single_map;

	float scaler;

private:
	void filterXYTheta(const std::vector<Pose<double>> robot_poses,
	                   std::vector<Pose<double>>&      filtered_poses);
	void drawMap(const std::vector<Pose<double>>& robot_poses);
	void drawHistogram(const std::vector<Pose<double>>& robot_poses);
	void predict(const std::vector<Pose<double>>& robot_poses);

	std::vector<Landmark<double>>
	          averagePrediction(const std::vector<Pose<double>>& robot_poses);
	Grid<int> histogramPrediction(const std::vector<Pose<double>>& robot_poses);

	Parameters         params;
	LandmarkProcessor* lprocessor;
	Pose<double>       prev_pose;
};
