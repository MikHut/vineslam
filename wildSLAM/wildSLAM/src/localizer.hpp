#pragma once

// Class objects
#include <landmark.hpp>
#include <pf.hpp>
#include <math/point3D.hpp>
#include <math/pose6D.hpp>

// std, eigen
#include <iostream>
#include <map>
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
	void process(const pose6D& odom, const std::vector<float>& bearings,
	             const std::vector<float>&             depths,
	             const std::map<int, Landmark<float>>& map);

	// Export the final pose resultant from the localization procedure
	pose6D getPose() const;

private:
	// Average particles pose
	pose6D average_pose;
	// Particle filter object
	PF* pf;
	// Input parameters
	int    n_particles;
	float cam_pitch;

	// Auxiliar function that normalizes an angle in the [-pi,pi] range
	float normalizeAngle(const float& angle)
	{
		return (std::fmod(angle + PI, 2 * PI) - PI);
	}
};
