#pragma once

#include <iostream>

// Parameters structure
struct Parameters
{
	double h_fov;       // Camera horizontal field of view (radians)
	double fx;          // Camera x focal lenght (pixels)
	double fy;          // Camera y focal lenght (pixels)
	double cx;          // Camera x component of principal point (pixels)
	double cy;          // Camera y component of principal point (pixels)
	int    width;       // Image width
	int    height;      // Image height
	double baseline;    // Stereo baseline between cameras (meters)
	double delta_D;     // Stereo matcher disparity error (pixels)
	double min_score;   // Trunk detector minimum threshold
	int    n_particles; // Number of particles of the particle filter

	std::string image_left;  // left image ROS topic
	std::string image_depth; // depth image ROS topic
	std::string odom_topic;  // odometry ROS topic
	std::string gps_topic;   // gps ROS topic
	std::string model;       // tflite model path
	std::string labels;      // detection labels path

	Parameters()
	{
		fx          = 692.95849609375;
		fy          = 636.41424560546;
		cx          = 692.95849609375;
		cy          = 382.80633544921;
		width       = 1280;
		height      = 720;
		min_score   = 0.5;
		delta_D     = 1.0;
		n_particles = 1000;
		h_fov       = 1.82;
		baseline    = 0.12;

		model       = "";
		labels      = "";
		image_left  = "";
		image_depth = "";
		odom_topic  = "";
		gps_topic   = "";
	}
};
