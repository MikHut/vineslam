#pragma once

// Classes
#include <kf.hpp>
#include <landmark.hpp>
#include <pose.hpp>

// ROS, std, eigen
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <cmath>
#include <numeric>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

class Mapper2D
{
public:
	// Class constructor
	// - Loads the parameters
	Mapper2D(const std::string& config_path);

	// Global function that handles all the mapping process
	void process(Pose<double>& pose, const std::vector<double>& bearings,
	             const std::vector<double>& depths,
	             const tf::Transform& cam2world, const std::vector<int>& labels);

	// Initializes the map
	// - Invocated only once to insert the first observations on the map
	void init(Pose<double>& pose, const std::vector<double>& bearings,
	          const std::vector<double>& depths, const std::vector<int>& labels);

	// Exports the current map to the high level ROS node
	std::map<int, Landmark<double>> getMap() const;

	// Map that contains
	// - the key id of each landmark
	// - a Landmark object that represents it
	std::map<int, Landmark<double>> map;
	// Array of Kalman Filters, one for each landmark
	std::vector<KF> filters;

private:
	// Input parameters
	double      baseline;
	double      delta_d;
	double      fx;
	std::string config_path;

	// Estimates landmark positions based on the current observations
	void predict(Pose<double>& pose, const std::vector<double>& bearings,
	             const std::vector<double>& depths,
	             const std::vector<int>&    labels);

	// Computes a local map, on robot's frame
	std::vector<Point<double>> local_map(Pose<double>&        pose,
	                                     const std::vector<double>& bearings,
	                                     const std::vector<double>& depths,
	                                     const tf::Transform&       cam2map);

	// Searches from correspondences between observations and landmarks
	// already mapped
	int findCorr(const Point<double>& l_pos, const Point<double>& r_pos);

	// Auxiliar function that normalizes an angle in the [-pi,pi] range
	double normalizeAngle(const double& angle)
	{
		return (std::fmod(angle + PI, 2 * PI) - PI);
	}

	// Auxiliar function that the disparity error using the disparity
	// noise model
	double dispError(const double& depth)
	{
		return pow(depth, 2) / (baseline * fx) * delta_d;
	}
};
