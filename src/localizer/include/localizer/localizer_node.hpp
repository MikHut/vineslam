#pragma once

#include "particle_filter.hpp"
#include "utils.hpp"
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

/* edgetpu detection API */
#include <detection/engine.h>
#include <examples/label_utils.h>
#include <examples/model_utils.h>
#include <test_utils.h>

class Localizer
{
public:
	Localizer(int argc, char** argv);


private:
	void imageListener(const sensor_msgs::ImageConstPtr& msg);

	ros::Subscriber img_subscriber;
	Parameters*     params;

	std::vector<Landmark<double>> landmarks;
	ParticleFilter*               pfilter;

	std::vector<int>                     input_tensor_shape;
	coral::DetectionEngine*              engine;
	std::unordered_map<int, std::string> labels;

	void loadParameters(const ros::NodeHandle& local_nh)
	{
		/* read launch file parameters */
		local_nh.getParam("/localizer/pose_topic", (*params).pose_topic);
		local_nh.getParam("/localizer/image_topic", (*params).image_topic);
		local_nh.getParam("/localizer/map_file", (*params).map_file);
		local_nh.getParam("/localizer/h_fov", (*params).h_fov);
		local_nh.getParam("/localizer/v_fov", (*params).v_fov);
		local_nh.getParam("/localizer/img_width", (*params).width);
		local_nh.getParam("/localizer/img_height", (*params).height);
		local_nh.getParam("/localizer/model_path", (*params).model);
		local_nh.getParam("/localizer/labels_path", (*params).labels);
		local_nh.getParam("/localizer/detector_th", (*params).min_score);
	}

	void loadLandmarks(const std::string& path)
	{
		std::string   line;
		std::ifstream file(path);

		if (file.is_open()) {
			while (std::getline(file, line)) {
				std::istringstream iss(line);
				std::string        s;

				iss >> s;
				int id = std::atoi(s.c_str());

				Point<double> pt;
				iss >> s;
				pt.x = std::atof(s.c_str());
				iss >> s;
				pt.y = std::atof(s.c_str());

				landmarks.push_back(Landmark<double>(id, pt));
			}
			file.close();
		}
		else
			std::cout << "Unable to read map file" << std::endl;
	}
};
