#pragma once

// Classes
#include <kf.hpp>
#include <landmark.hpp>
#include <math/ellipse2D.hpp>
#include <math/point3D.hpp>
#include <math/pose6D.hpp>

// std, eigen
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <numeric>
#include <yaml-cpp/yaml.h>

class Mapper2D
{
public:
	// Class constructor
	// - Loads the parameters
	Mapper2D(const std::string& config_path);

	// Global function that handles all the mapping process
	void process(pose6D& pose, const std::vector<float>& bearings,
	             const std::vector<float>& depths,
	             const std::vector<int>&   labels);

	// Initializes the map
	// - Invocated only once to insert the first observations on the map
	void init(pose6D& pose, const std::vector<float>& bearings,
	          const std::vector<float>& depths, const std::vector<int>& labels);

	// Computes a local map, on robot's frame
	std::vector<point3D> local_map(pose6D&                   pose,
	                               const std::vector<float>& bearings,
	                               const std::vector<float>& depths);

	// Exports the current map to the high level ROS node
	std::map<int, Landmark<float>> getMap() const;

	// Map that contains
	// - the key id of each landmark
	// - a Landmark object that represents it
	std::map<int, Landmark<float>> map;
	// Array of Kalman Filters, one for each landmark
	std::vector<KF> filters;

private:
	// Input parameters
	float       baseline;
	float       delta_d;
	float       fx;
	std::string config_path;

	// Estimates landmark positions based on the current observations
	void predict(pose6D& pose, const std::vector<float>& bearings,
	             const std::vector<float>& depths,
	             const std::vector<int>&   labels);

	// Computes a local map, on robot's frame
	std::vector<Point<double>> local_map(Pose<double>&        pose,
	                                     const std::vector<double>& bearings,
	                                     const std::vector<double>& depths,
	                                     const tf::Transform&       cam2map);

	// Searches from correspondences between observations and landmarks
	// already mapped
	int findCorr(const point3D& l_pos, const point3D& r_pos);

	// Auxiliar function that normalizes an angle in the [-pi,pi] range
	float normalizeAngle(const float& angle)
	{
		return (std::fmod(angle + PI, 2 * PI) - PI);
	}

	// Auxiliar function that the disparity error using the disparity
	// noise model
	float dispError(const float& depth)
	{
		return pow(depth, 2) / (baseline * fx) * delta_d;
	}
};
