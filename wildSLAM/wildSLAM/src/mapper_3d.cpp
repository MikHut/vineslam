#include "mapper_3d.hpp"

Mapper3D::Mapper3D(const Parameters& params) : params(params) {}

void Mapper3D::init() {}

void Mapper3D::process(const float* depths)
{
	// Define the minimum and maximum levels of disparity to consider
	double range_min = 0.01;
	double range_max = 10.0;

	// Erase the last point cloud
	pcl.clear();

  // Loop over the entire disparity map image
	for (int i = 0; i < params.width; i++) {
		for (int j = 0; j < params.height; j++) {
      // Calculate the 1D index of the disparity array
			int idx = i + params.width * j;
      // Check if the current disparity value is valid
			if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
			    depths[idx] < range_max) {
	      // Compute the 3D point considering the disparity and
        // the camera parameters
				Point<double> point3D;
				point3D.z = depths[idx];
				point3D.x = (double)(i - params.cx) * (point3D.z / params.fx);
				point3D.y = (double)(j - params.cy) * (point3D.z / params.fy);

        // Fill the point cloud array
				pcl.push_back(point3D);
			}
		}
	}
}

std::vector<Point<double>> Mapper3D::getPointCloud() const
{
	return pcl;
}
