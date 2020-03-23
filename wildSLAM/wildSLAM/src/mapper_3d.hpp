#pragma once

#include <params.hpp>
#include <pose.hpp>

class Mapper3D
{
public:
	// Class constructor - receives and saves the system
	// parameters
	Mapper3D(const Parameters& params);

	// Initialization function
	void init();

	// Handles the loop process of the 3D mapper
	void process(const float* depths);
  // Returns the current point cloud of the object
  std::vector<Point<double>> getPointCloud() const;

private:
  // Paramaters object
	Parameters params;
  // Point cloud variable
  std::vector<Point<double>> pcl;
};
