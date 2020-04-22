#pragma once

// Class objects
#include <landmark.hpp>
#include <math/point3D.hpp>
#include <math/pose6D.hpp>
#include <pf.hpp>
#include <mapper_3d.hpp>

// std, eigen
#include <iostream>
#include <map>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <yaml-cpp/yaml.h>

#define PI 3.14159265359

class Localizer
{
public:
	// Class constructor
	Localizer(const std::string& config_path);

	// Initializes the particle filter with the number of particles
	// and the first odometry pose
	void init(const pose6D& initial_pose);

	// Global function that handles all the localization process
	// Arguments:
	// - odom:         wheel odometry pose6D
	// - bearings2D:   bearing observations to process 2D localization
	// - depths2D:     depth observations to process 2D localization
	// - map:          2D semantic feature map
	// - depths3D:     raw sensor depths to process 3D localization
	// - intensities:  RGB color intensities of the raw depths3D
	// - dets:         bounding boxes to isolate trunks on 3D mapping/localization
	void process(const pose6D& odom, const std::vector<float>& bearings2D,
	             const std::vector<float>&             depths2D,
	             const std::map<int, Landmark<float>>& map, float* depths3D,
	             const vision_msgs::Detection2DArray&       dets);

	// Export the final pose resultant from the localization procedure
	pose6D getPose() const;
	// Export the all the poses referent to all the particles
	void getParticles(std::vector<pose6D>& in) const;

	// Set sensor data to the particle filter
	void setSensorData(float*                                     raw_depths,
	                   const std::vector<std::array<uint8_t, 3>>& intensities,
	                   const vision_msgs::Detection2DArray&       dets);

private:
	// Average particles pose
	pose6D average_pose;
	// Particle filter object
	PF* pf;
	// Input parameters
	int   n_particles;
	float cam_pitch;
	// Input parameters file name
	std::string config_path;

	// Auxiliar function that normalizes an angle in the [-pi,pi] range
	float normalizeAngle(const float& angle)
	{
		return (std::fmod(angle + PI, 2 * PI) - PI);
	}
};
