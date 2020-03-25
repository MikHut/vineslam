#pragma once

#include <pose.hpp>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <yaml-cpp/yaml.h>

class Mapper3D
{
public:
	// Class constructor - receives and saves the system
	// parameters
	Mapper3D(const std::string& config_path);

	// Initialization function
	void init();

	// Handles the loop process of the 3D mapper
	void process(const float* depths, const vision_msgs::Detection2DArray& dets);
	// Returns the current raw point cloud
	std::vector<Point<double>> getRawPointCloud() const;
	// Returns the current trunk point cloud
	std::vector<Point<double>> getTrunkPointCloud() const;

private:
	// Function that handles the design of the raw 3D map
	void buildRawMap(const float* depths);
	// Function that handles the design of the vine trunks 3D map
	void buildTrunkMap(const float*                         depths,
	                   const vision_msgs::Detection2DArray& dets);

	// Point cloud variables
	std::vector<Point<double>> raw_pcl;
	std::vector<Point<double>> trunk_pcl;

	// Camera info parameters
	double img_width;
	double img_height;
	double cam_height;
	double fx;
	double fy;
	double cx;
	double cy;
};
