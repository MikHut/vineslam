#pragma once

#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <yaml-cpp/yaml.h>

// OCTOMAP
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>

typedef octomap::point3d point3D;
typedef octomath::Quaternion quaternion;

// Typedefs
typedef octomap::ColorOcTree OcTreeT;

class Mapper3D
{
public:
	// Class constructor - receives and saves the system
	// parameters
	Mapper3D(const std::string& config_path);

	// Initialization function
	void init();

	// Handles the loop process of the 3D mapper
	void process(const float* depths, const point3D& sensor_origin,
	             const quaternion&                    sensor_rot,
	             const vision_msgs::Detection2DArray& dets);
	// Returns the current raw point cloud
	OcTreeT getRawPointCloud() const;
	// Returns the current trunk point cloud
	OcTreeT getTrunkPointCloud() const;

private:
	// Function that handles the design of the raw 3D map
	void buildRawMap(const float* depths, const point3D& sensor_origin,
	                 const quaternion& sensor_rot);
	// Function that handles the design of the vine trunks 3D map
	void buildTrunkMap(const float* depths, const point3D& sensor_origin,
	                   const quaternion&                    sensor_rot,
	                   const vision_msgs::Detection2DArray& dets);
	// Creates an OctoMap using the Octree structure
	void createOctoMap(const point3D&              sensor_origin,
	                   const std::vector<point3D>& pcl);

	// TODO (André Aguiar): Misses documentation for this function
	inline static void updateMinKey(const octomap::OcTreeKey& in,
	                                octomap::OcTreeKey&       min)
	{
		for (unsigned i = 0; i < 3; ++i)
			min[i] = std::min(in[i], min[i]);
	};

	// TODO (André Aguiar): Misses documentation for this function
	inline static void updateMaxKey(const octomap::OcTreeKey& in,
	                                octomap::OcTreeKey&       max)
	{
		for (unsigned i = 0; i < 3; ++i)
			max[i] = std::max(in[i], max[i]);
	};

	// Point cloud variables
	std::vector<point3D> raw_pcl;
	std::vector<point3D> trunk_pcl;

	// Octree variables
	OcTreeT*           octree;
	octomap::KeyRay    key_ray;
	octomap::OcTreeKey update_BBXMin;
	octomap::OcTreeKey update_BBXMax;

	// Octree parameters
	double res;
	double prob_hit;
	double prob_miss;
	double thresh_min;
	double thresh_max;
	double max_range;

	// Camera info parameters
	double img_width;
	double img_height;
	double cam_height;
	double fx;
	double fy;
	double cx;
	double cy;
};
