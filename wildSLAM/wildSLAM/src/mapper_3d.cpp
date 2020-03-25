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

void Mapper3D::process(const float*                         depths,
                       const vision_msgs::Detection2DArray& dets)
{
	// Build raw 3D map
	buildRawMap(depths);
	// Build trunks 3D map
	buildTrunkMap(depths, dets);
}

void Mapper3D::buildRawMap(const float* depths)
{
	// Define the minimum and maximum levels of disparity to consider
	double range_min = 0.01;
	double range_max = 10.0;

	// Erase the last point cloud
	raw_pcl.clear();

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
				raw_pcl.push_back(point3D);
			}
		}
	}
}

void Mapper3D::buildTrunkMap(const float*                         depths,
                             const vision_msgs::Detection2DArray& dets)
{
	// Define the minimum and maximum levels of disparity to consider
	double range_min = 0.01;
	double range_max = 10.0;

	// Erase the last point cloud
	trunk_pcl.clear();

	// Loop over all the bounding boxes
	for (size_t n = 0; n < dets.detections.size(); n++) {
		// Load a single bounding box detection
		vision_msgs::BoundingBox2D m_bbox = dets.detections[n].bbox;
		// Compute the limites of the bounding boxes
		double xmin = m_bbox.center.x - m_bbox.size_x / 2;
		double xmax = m_bbox.center.x + m_bbox.size_x / 2;
		double ymin = m_bbox.center.y - m_bbox.size_y / 2;
		double ymax = m_bbox.center.y + m_bbox.size_y / 2;

		// Initialize and compute average depth inside the bbox
		double mean_depth = 0;
		int    it         = 0;
		for (int x = xmin; x < xmax; x++) {
			for (int y = ymin; y < ymax; y++) {
				int idx = x + img_width * y;
				// Check if the current disparity value is valid
				if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
				    depths[idx] < range_max) {
            mean_depth += depths[idx];
            it++;
        }
			}
		}
    mean_depth /= (double)it;


		for (int x = xmin; x < xmax; x++) {
			for (int y = ymin; y < ymax; y++) {
				int idx = x + img_width * y;
				// Check if the current disparity value is valid
				if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
				    depths[idx] < mean_depth) {
					// Compute the 3D point considering the disparity and
					// the camera parameters
					Point<double> point3D;
					point3D.z = depths[idx];
					point3D.x = (double)(x - cx) * (point3D.z / fx);
					point3D.y = (double)(y - cy) * (point3D.z / fy) - cam_height;

					// Fill the point cloud array
					trunk_pcl.push_back(point3D);
				}
			}
		}
	}
}

std::vector<Point<double>> Mapper3D::getRawPointCloud() const
{
	return raw_pcl;
}

std::vector<Point<double>> Mapper3D::getTrunkPointCloud() const
{
	return trunk_pcl;
}
