#pragma once

#include "kf.hpp"
#include "utils.hpp"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <iostream>
#include <map>
#include <math.h>
#include <numeric>

class Estimator
{
public:
	// Class constructor
	// - Loads the parameters
	Estimator(const Parameters& params);

	// Global function that handles all the estimation process
	void process(const Pose<double>& odom, const std::vector<double>& bearings,
	             const std::vector<double>& depths);
  
	// Initializes the map
	// - Invocated only once to insert the first observations on the map
	void init(const Pose<double>& odom, const std::vector<double>& bearings,
	          const std::vector<double>& depths);

  // Exports the current map to the high level ROS node
  std::map<int, Landmark<double>> getMap() const;

	// Map that contains
	// - the landmarks estimations
	// - the number of observations of each landmark
	std::map<int, Landmark<double>> map;
	// Array of Kalman Filters, one for each landmark
	std::vector<KF> filters;

private:
	// Estimates landmark positions based on the current observations
	void predict(const Pose<double>& odom, const std::vector<double>& bearings,
	             const std::vector<double>& depths);

	// Searches from correspondences between observations and landmarks
	// already mapped
	int findCorr(const Point<double>& pos);

	Parameters params;
};
