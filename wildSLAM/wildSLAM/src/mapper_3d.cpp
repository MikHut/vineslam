#include "mapper_3d.hpp"

using namespace octomap;

Mapper3D::Mapper3D(const std::string& config_path)
{
	// Load configuration file
	YAML::Node config = YAML::LoadFile(config_path.c_str());

	// Load camera info parameters
	img_width  = config["camera_info"]["img_width"].as<double>();
	img_height = config["camera_info"]["img_height"].as<double>();
	cam_height = config["camera_info"]["cam_height"].as<double>();
	fx         = config["camera_info"]["fx"].as<double>();
	fy         = config["camera_info"]["fy"].as<double>();
	cx         = config["camera_info"]["cx"].as<double>();
	cy         = config["camera_info"]["cy"].as<double>();

	// Load octree parameters
	res        = config["mapper3D"]["resolution"].as<double>();
	prob_hit   = config["mapper3D"]["hit"].as<double>();
	prob_miss  = config["mapper3D"]["miss"].as<double>();
	thresh_min = config["mapper3D"]["thresh_min"].as<double>();
	thresh_max = config["mapper3D"]["thresh_max"].as<double>();
	max_range  = config["mapper3D"]["max_range"].as<double>();
}

void Mapper3D::init()
{
	// Initialize Octree
	octree = new OcTreeT(res);
	(*octree).setProbHit(prob_hit);
	(*octree).setProbMiss(prob_miss);
	(*octree).setClampingThresMin(thresh_min);
	(*octree).setClampingThresMax(thresh_max);
}

void Mapper3D::process(const float*                               depths,
                       const std::vector<std::array<uint8_t, 3>>& intensities,
                       const point3D&                             sensor_origin,
                       const octomath::Quaternion&                sensor_rot,
                       const vision_msgs::Detection2DArray&       dets)
{
	// Build raw 3D map
	//buildRawMap(depths, intensities, sensor_origin, sensor_rot);
	// Build trunks 3D map
	buildTrunkMap(depths, intensities, sensor_origin, sensor_rot, dets);
}

void Mapper3D::buildRawMap(
    const float* depths, const std::vector<std::array<uint8_t, 3>>& intensities,
    const point3D& sensor_origin, const octomath::Quaternion& sensor_rot)
{
	// Define the minimum and maximum levels of disparity to consider
	double range_min = 0.01;
	double range_max = 10.0;

	// Declare 3D point and intensities vectors
	std::vector<point3D>                pts_array;
	std::vector<std::array<uint8_t, 3>> ints_array;

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
				point3D point;
				point.z() = depths[idx];
				point.x() = (double)(i - cx) * (point.z() / fx);
				point.y() = (double)(j - cy) * (point.z() / fy) - cam_height;

				// Rotate camera axis to map axis
				point3D point_cam;
				point_cam.x() = +point.z();
				point_cam.y() = -point.x();
				point_cam.z() = -point.y();

				// Camera to map point cloud conversion
				std::vector<double> rot_matrix;
				sensor_rot.toRotMatrix(rot_matrix);
				point3D point_map;
				point_map.x() = point_cam.x() * rot_matrix[0] +
				                point_cam.y() * rot_matrix[1] +
				                point_cam.z() * rot_matrix[2] + sensor_origin.x();
				point_map.y() = point_cam.x() * rot_matrix[3] +
				                point_cam.y() * rot_matrix[4] +
				                point_cam.z() * rot_matrix[5] + sensor_origin.y();
				point_map.z() = point_cam.x() * rot_matrix[6] +
				                point_cam.y() * rot_matrix[7] +
				                point_cam.z() * rot_matrix[8] + sensor_origin.z();


				// Fill the point cloud array
				pts_array.push_back(point_map);
				ints_array.push_back(intensities[idx]);
			}
		}
	}

	// Create octomap using the build PCL
	createOctoMap(sensor_origin, pts_array, ints_array);
}

void Mapper3D::buildTrunkMap(
    const float* depths, const std::vector<std::array<uint8_t, 3>>& intensities,
    const point3D& sensor_origin, const octomath::Quaternion& sensor_rot,
    const vision_msgs::Detection2DArray& dets)
{
	// Define the minimum and maximum levels of disparity to consider
	double range_min = 0.01;
	double range_max = 10.0;

	// Declare 3D point and intensities vectors
	std::vector<point3D>                pts_array;
	std::vector<std::array<uint8_t, 3>> ints_array;

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
					point3D point;
					point.z() = depths[idx];
					point.x() = (double)(x - cx) * (point.z() / fx);
					point.y() = (double)(y - cy) * (point.z() / fy) - cam_height;

					// Rotate camera axis to map axis
					point3D point_cam;
					point_cam.x() = +point.z();
					point_cam.y() = -point.x();
					point_cam.z() = -point.y();

					// Camera to map point cloud conversion
					std::vector<double> rot_matrix;
					sensor_rot.toRotMatrix(rot_matrix);
					point3D point_map;
					point_map.x() = point_cam.x() * rot_matrix[0] +
					                point_cam.y() * rot_matrix[1] +
					                point_cam.z() * rot_matrix[2] + sensor_origin.x();
					point_map.y() = point_cam.x() * rot_matrix[3] +
					                point_cam.y() * rot_matrix[4] +
					                point_cam.z() * rot_matrix[5] + sensor_origin.y();
					point_map.z() = point_cam.x() * rot_matrix[6] +
					                point_cam.y() * rot_matrix[7] +
					                point_cam.z() * rot_matrix[8] + sensor_origin.z();
					// Fill the point cloud array
					pts_array.push_back(point_map);
					ints_array.push_back(intensities[idx]);
				}
			}
		}
	}

	// Create octomap using the built PCL
	createOctoMap(sensor_origin, pts_array, ints_array);
}

void Mapper3D::createOctoMap(const point3D&              sensor_origin,
                             const std::vector<point3D>& pcl,
                             const std::vector<std::array<uint8_t, 3>>& ints)
{
	// Declare cells structures to fill
	KeySet occupied_cells;

	// Loop over all the point in the point cloud
	for (size_t i = 0; i < pcl.size(); i++) {
		// Load the current point
		point3D point = pcl[i];

		// Check max range
		if ((max_range < 0.0) || ((point - sensor_origin).norm() <= max_range)) {

			OcTreeKey key;
			if ((*octree).coordToKeyChecked(point, key)) {
				occupied_cells.insert(key);

				updateMinKey(key, update_BBXMin);
				updateMinKey(key, update_BBXMax);

				uint8_t r = ints[i][0];
				uint8_t g = ints[i][1];
				uint8_t b = ints[i][2];

				(*octree).averageNodeColor(key, r, g, b);
			}
		}
	}

	// Mark all occupied cells
	for (KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end();
	     it != end; it++) {
		(*octree).updateNode(*it, true);
  }
}

OcTreeT* Mapper3D::getRawPointCloud()
{
	return octree;
}

OcTreeT* Mapper3D::getTrunkPointCloud()
{
	return octree;
}
