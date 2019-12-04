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
	cv::Mat histogram;

	float scaler;
	int   map_width;
	int   map_heigth;

private:
	void filterXYTheta(const std::vector<Pose<double>> robot_poses,
	                   std::vector<Pose<double>>&      filtered_poses);
	void drawMap(const std::vector<Pose<double>>& robot_poses);
	void drawHistogram(const std::vector<Pose<double>>& robot_poses);
	void predict(const std::vector<Pose<double>>& robot_poses);

	std::vector<Landmark<double>>
	averagePrediction(const std::vector<Pose<double>>& robot_poses);
	std::vector<Landmark<double>>
	histogramPrediction(const std::vector<Pose<double>>& robot_poses);
	std::vector<Landmark<double>>
	kfPrediction(const std::vector<Pose<double>>& robot_poses);

	Point<double> processObsv(const Landmark<double>& l, const int& it,
	                          const Pose<double>& delta_p);

	std::vector<Point<double>> initMap(const int& N_x, const int& N_y);
	Point<double>              correspond(const Point<double>& r, const int& col,
	                                      const std::vector<Point<double>>& map);

	double columnToTheta(const int& col)
	{
		return (-params.h_fov / params.width) * (params.width / 2 - col);
	}

	Parameters         params;
	LandmarkProcessor* lprocessor;
	Pose<double>       prev_pose;
};
