#pragma once

#include <iostream>

// Parameters structure
struct Parameters
{
	double h_fov;       // Camera horizontal field of view (radians)
	double f_length;    // Camera focal lenght (pixels)
	int    width;       // Image width
	int    height;      // Image height
	double baseline;    // Stereo baseline between cameras (meters)
	double delta_D;     // Stereo matcher disparity error (pixels)
	double min_score;   // Trunk detector minimum threshold
	int    n_particles; // Number of particles of the particle filter

	std::string image_left;  // left image ROS topic
	std::string image_depth; // depth image ROS topic
	std::string odom_topic;  // odometry ROS topic
	std::string model;       // tflite model path
	std::string labels;      // detection labels path

	Parameters()
	{
		f_length    = 0.1;
		width       = 1280;
		height      = 960;
		min_score   = 0.5;
		delta_D     = 0.2;
		n_particles = 1000;

		model       = "";
		labels      = "";
		image_left  = "";
		image_depth = "";
		odom_topic  = "";
	}
};
