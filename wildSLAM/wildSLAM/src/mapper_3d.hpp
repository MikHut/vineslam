#pragma once

#include <pose.hpp>
#include <yaml-cpp/yaml.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>

class Mapper3D
{
public:
	// Class constructor - receives and saves the system
	// parameters
	Mapper3D(const std::string& config_path);

	// Initialization function
	void init();

	// Handles the loop process of the 3D mapper
	void process(const float* depths);
  // Returns the current point cloud of the object
  std::vector<Point<double>> getPointCloud() const;

private:
  // Point cloud variable
  std::vector<Point<double>> pcl;

  // Camera info parameters
  double img_width;
  double img_height;
  double cam_height;
  double fx;
  double fy;
  double cx;
  double cy;
};
