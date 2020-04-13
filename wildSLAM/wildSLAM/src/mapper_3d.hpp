#pragma once

#include <iostream>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <yaml-cpp/yaml.h>

// OCTOMAP
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>

// math
#include <math/point3D.hpp>
#include <math/pose6D.hpp>

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
	void process(const float*                               depths,
	             const std::vector<std::array<uint8_t, 3>>& intensities,
	             pose6D&                                    sensor_origin,
	             const vision_msgs::Detection2DArray&       dets);
	// Returns the current raw point cloud
	OcTreeT* getRawPointCloud() const;
	// Returns the current trunk point cloud
	OcTreeT* getTrunkPointCloud() const;

private:
	// Function that handles the design of the raw 3D map
	void buildRawMap(const float*                               depths,
	                 const std::vector<std::array<uint8_t, 3>>& intensities,
	                 pose6D&                                    sensor_origin);
	// Function that handles the design of the vine trunks 3D map
	void buildTrunkMap(const float*                               depths,
	                   const std::vector<std::array<uint8_t, 3>>& intensities,
	                   pose6D&                                    sensor_origin,
	                   const vision_msgs::Detection2DArray&       dets);
	// Creates an OctoMap using the Octree structure
	void createOctoMap(pose6D& sensor_origin, const std::vector<point3D>& pcl,
	                   const std::vector<std::array<uint8_t, 3>>& ints);

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

	// Octree variables
	OcTreeT*           octree;
	octomap::KeyRay    key_ray;
	octomap::OcTreeKey update_BBXMin;
	octomap::OcTreeKey update_BBXMax;

	// Octree parameters
	float res;
	float prob_hit;
	float prob_miss;
	float thresh_min;
	float thresh_max;
	float max_range;

	// Camera info parameters
	float img_width;
	float img_height;
	float cam_height;
	float fx;
	float fy;
	float cx;
	float cy;
};
