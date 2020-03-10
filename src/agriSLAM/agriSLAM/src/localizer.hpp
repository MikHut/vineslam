#pragma once

// Class objects
#include <landmark.hpp>
#include <pf.hpp>
#include <utils.hpp>

// ROS, std, eigen
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <tf/transform_listener.h>

class Localizer
{
public:
	// Class constructor
	Localizer(const Parameters& params);

  // Initializes the particle filter with the number of particles
  // and the first odometry pose
  void init(const Pose<double>& initial_pose);

	// Global function that handles all the localization process
	void process(const Pose<double>& odom, const std::vector<double>& bearings,
	             const std::vector<double>&             depths,
	             const std::map<int, Landmark<double>>& map);

	// Export the matrix that transforms the camera into the world's
	// referential frame in ROS mode
	tf::Transform getTf() const;

	// Export the array of poses in ROS mode relative to all particles
	geometry_msgs::PoseArray getPoseArray() const;

	// Export the final pose resultant from the localization procedure
	Pose<double> getPose() const;

private:
	// ROS Homogeneous transformation from camera to world
	tf::Transform cam2map;
	// ROS Array of poses relative to all particles
	geometry_msgs::PoseArray poses;
  // Average particles pose
  Pose<double> average_pose;
	// Particle filter object
	PF* pf;
	// Parameters inputs
	Parameters params;

	// Auxiliar function that normalizes an angle in the [-pi,pi] range
	double normalizeAngle(const double& angle)
	{
		return (std::fmod(angle + PI, 2 * PI) - PI);
	}
};
