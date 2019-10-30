#pragma once

#ifndef Q_MOC_RUN
#include "../../utils/qnode.hpp"
#include <ros/ros.h>
#endif
#include "landmark_processor.hpp"
#include "mapper.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

/* edgetpu detection API */
#include <detection/engine.h>
#include <examples/label_utils.h>
#include <examples/model_utils.h>
#include <test_utils.h>

class Mapper : public QNode
{

public:
	Mapper(int argc, char** argv);
	virtual ~Mapper() {}
	void run();
	void rosCommsInit();
	void retrieveLog(std::string& log);
	void constructMap();

	const cv::Mat exportMap();

private:
	void imageListener(const sensor_msgs::ImageConstPtr& msg);
	void poseListener(const geometry_msgs::PoseStampedConstPtr& msg);

	ros::Subscriber img_subscriber;
	ros::Subscriber pose_subscriber;

	bool init;

	Pose<double>              last_pose;
	std::vector<Pose<double>> all_poses;

	std_msgs::Header    scan_header;
	geometry_msgs::Pose slam_pose;

	Estimator*         estimator;
	LandmarkProcessor* lprocessor;
	Parameters*        params;

	void loadParameters(const ros::NodeHandle& local_nh)
	{
		/* read launch file parameters */
		local_nh.getParam("/mapper/h_fov", (*params).h_fov);
		local_nh.getParam("/mapper/v_fov", (*params).v_fov);
		local_nh.getParam("/mapper/img_width", (*params).width);
		local_nh.getParam("/mapper/img_height", (*params).height);
		local_nh.getParam("/mapper/match_box", (*params).match_box);
		local_nh.getParam("/mapper/filter_window", (*params).filter_window);
		local_nh.getParam("/mapper/mapper_inc", (*params).mapper_inc);
		local_nh.getParam("/mapper/model_path", (*params).model);
		local_nh.getParam("/mapper/labels", (*params).labels);
	}
};
