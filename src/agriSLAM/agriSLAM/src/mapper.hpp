#pragma once

// Classes
#include <kf.hpp>
#include <landmark.hpp>
#include <utils.hpp>

// ROS, std, eigen
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <math.h>
#include <numeric>
#include <ros/ros.h>

class Mapper
{
public:
	// Class constructor
	// - Loads the parameters
	Mapper(const Parameters& params);

	// Global function that handles all the mapping process
	void process(const Pose<double>& odom, const std::vector<double>& bearings,
	             const std::vector<double>&       depths,
	             const std::vector<SemanticInfo>& info);

	// Initializes the map
	// - Invocated only once to insert the first observations on the map
	void init(const Pose<double>& odom, const std::vector<double>& bearings,
	          const std::vector<double>&       depths,
	          const std::vector<SemanticInfo>& info);

	// Exports the current map to the high level ROS node
	std::map<int, Landmark<double>> getMap() const;

	// Map that contains
	// - the key id of each landmark
	// - a Landmark object that represents it
	std::map<int, Landmark<double>> map;
	// Array of Kalman Filters, one for each landmark
	std::vector<KF> filters;

private:
	// Estimates landmark positions based on the current observations
	void predict(const Pose<double>& odom, const std::vector<double>& bearings,
	             const std::vector<double>&       depths,
	             const std::vector<SemanticInfo>& info);

	// Searches from correspondences between observations and landmarks
	// already mapped
	int findCorr(const Point<double>& l_pos, const Point<double>& r_pos);
	// Calculates the disparity error using the disparity noise model
	double dispError(const double& depth)
	{
		return pow(depth, 2) / (params.baseline * params.f_length) * params.delta_D;
	}

	Parameters params;
};
