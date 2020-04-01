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
                       pose6D&                                    sensor_origin,
                       const vision_msgs::Detection2DArray&       dets)
{
	// Build raw 3D map
	// buildRawMap(depths, sensor_origin);
	// Build trunks 3D map
	buildTrunkMap(depths, intensities, sensor_origin, dets);
}

void Mapper3D::buildRawMap(
    const float* depths, const std::vector<std::array<uint8_t, 3>>& intensities,
    pose6D& sensor_origin)
{
	// Define the minimum and maximum levels of disparity to consider
	float range_min = 0.01;
	float range_max = 10.0;

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
				point.z = depths[idx];
				point.x = (float)(i - cx) * (point.z / fx);
				point.y = (float)(j - cy) * (point.z / fy) - cam_height;

				// Rotate camera axis to map axis
				point3D point_cam;
				point_cam.x = +point.z;
				point_cam.y = -point.x;
				point_cam.z = -point.y;

				// Camera to map point cloud conversion
				std::vector<float> rot_matrix;
				sensor_origin.toRotMatrix(rot_matrix);
				point3D point_map;
				point_map.x = point_cam.x * rot_matrix[0] +
				              point_cam.y * rot_matrix[1] +
				              point_cam.z * rot_matrix[2] + sensor_origin.x;
				point_map.y = point_cam.x * rot_matrix[3] +
				              point_cam.y * rot_matrix[4] +
				              point_cam.z * rot_matrix[5] + sensor_origin.y;
				point_map.z = point_cam.x * rot_matrix[6] +
				              point_cam.y * rot_matrix[7] +
				              point_cam.z * rot_matrix[8] + sensor_origin.z;


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
    pose6D& sensor_origin, const vision_msgs::Detection2DArray& dets)
{
	// Define the minimum and maximum levels of disparity to consider
	float range_min = 0.01;
	float range_max = 10.0;

	// Declare 3D point and intensities vectors
	std::vector<point3D>                pts_array;
	std::vector<std::array<uint8_t, 3>> ints_array;

	// Loop over all the bounding boxes
	for (size_t n = 0; n < dets.detections.size(); n++) {
		// Load a single bounding box detection
		vision_msgs::BoundingBox2D m_bbox = dets.detections[n].bbox;
		// Compute the limites of the bounding boxes
		float xmin = m_bbox.center.x - m_bbox.size_x / 2;
		float xmax = m_bbox.center.x + m_bbox.size_x / 2;
		float ymin = m_bbox.center.y - m_bbox.size_y / 2;
		float ymax = m_bbox.center.y + m_bbox.size_y / 2;

		// Initialize and compute average depth inside the bbox
		float mean_depth = 0;
		int   it         = 0;
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
		mean_depth /= (float)it;


		for (int x = xmin; x < xmax; x++) {
			for (int y = ymin; y < ymax; y++) {
				int idx = x + img_width * y;
				// Check if the current disparity value is valid
				if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
				    depths[idx] < mean_depth) {
					// Compute the 3D point considering the disparity and
					// the camera parameters
					point3D point;
					point.z = depths[idx];
					point.x = (float)(x - cx) * (point.z / fx);
					point.y = (float)(y - cy) * (point.z / fy) - cam_height;

					// Rotate camera axis to map axis
					point3D point_cam;
					point_cam.x = +point.z;
					point_cam.y = -point.x;
					point_cam.z = -point.y;

					// Camera to map point cloud conversion
					std::vector<float> rot_matrix;
					sensor_origin.toRotMatrix(rot_matrix);
					point3D point_map;
					point_map.x = point_cam.x * rot_matrix[0] +
					              point_cam.y * rot_matrix[1] +
					              point_cam.z * rot_matrix[2] + sensor_origin.x;
					point_map.y = point_cam.x * rot_matrix[3] +
					              point_cam.y * rot_matrix[4] +
					              point_cam.z * rot_matrix[5] + sensor_origin.y;
					point_map.z = point_cam.x * rot_matrix[6] +
					              point_cam.y * rot_matrix[7] +
					              point_cam.z * rot_matrix[8] + sensor_origin.z;

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

void Mapper3D::createOctoMap(pose6D&                     sensor_origin,
                             const std::vector<point3D>& pcl,
                             const std::vector<std::array<uint8_t, 3>>& ints)
{
	// Declare cells structures to fill
	KeySet occupied_cells;
	KeySet free_cells;

	// Convert sensor origin to octomap format
	octomap::point3d m_sensor_origin(sensor_origin.x, sensor_origin.y,
	                                 sensor_origin.z);

	// Loop over all the point in the point cloud
	for (size_t i = 0; i < pcl.size(); i++) {
		// Load the current point
		point3d point(pcl[i].x, pcl[i].y, pcl[i].z);

		// Check max range
		if ((max_range < 0.0) || ((point - m_sensor_origin).norm() <= max_range)) {

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

OcTreeT *Mapper3D::getRawPointCloud() const
{
	return octree;
}

OcTreeT *Mapper3D::getTrunkPointCloud() const
{
	return octree;
}
