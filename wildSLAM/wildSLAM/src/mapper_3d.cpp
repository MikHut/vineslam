#include "mapper_3d.hpp"

Mapper3D::Mapper3D(const std::string& config_path)
{
	// Load camera info parameters
	YAML::Node config = YAML::LoadFile(config_path.c_str());
	img_width         = config["camera_info"]["img_width"].as<double>();
	img_height        = config["camera_info"]["img_height"].as<double>();
	cam_height        = config["camera_info"]["cam_height"].as<double>();
	fx                = config["camera_info"]["fx"].as<double>();
	fy                = config["camera_info"]["fy"].as<double>();
	cx                = config["camera_info"]["cx"].as<double>();
	cy                = config["camera_info"]["cy"].as<double>();
}

void Mapper3D::init() {}

void Mapper3D::process(const float* depths)
{
	// Define the minimum and maximum levels of disparity to consider
	double range_min = 0.01;
	double range_max = 10.0;

	// Erase the last point cloud
	pcl.clear();

	// Loop over the entire disparity map image
	for (int i = 0; i < img_width; i++) {
		for (int j = 0; j < img_height; j++) {
			// Calculate the 1D index of the disparity array
			int idx = i + img_width * j;
			// Check if the current disparity value is valid
			if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
			    depths[idx] < range_max) {
				// Compute the 3D point considering the disparity and
				// the camera parameters
				Point<double> point3D;
				point3D.z = depths[idx];
				point3D.x = (double)(i - cx) * (point3D.z / fx);
				point3D.y = (double)(j - cy) * (point3D.z / fy) - cam_height;

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
