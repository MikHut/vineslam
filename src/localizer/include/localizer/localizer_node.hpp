#pragma once

#include "utils.hpp"
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class Localizer
{
public:
	Localizer(int argc, char** argv);


private:
	void imageListener(const sensor_msgs::ImageConstPtr& msg);

	ros::Subscriber img_subscriber;
	Parameters*     params;

	std::vector<Landmark<double>> landmarks;

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
