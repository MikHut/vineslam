#pragma once

#ifndef Q_MOC_RUN
#include "../../utils/qnode.hpp"
#include <ros/ros.h>
#endif
#include "landmark_processor.hpp"
#include "mapper.hpp"
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

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
	void boxListener(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);
	void poseListener(const geometry_msgs::PoseStampedConstPtr& msg);
	ros::Subscriber box_subscriber;
	ros::Subscriber pose_subscriber;

	bool init;

	Pose<double>              last_pose;
	std::vector<Pose<double>> all_poses;

	std_msgs::Header                scan_header;
	geometry_msgs::Pose             slam_pose;
	darknet_ros_msgs::BoundingBoxes bounding_boxes;

	Estimator*         estimator;
	LandmarkProcessor* lprocessor;
	Parameters*        params;

	double h_fov;
	double v_fov;
	double cam_height;
	int    width;
	int    heigth;
	int    resolution;
	int    match_box;

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
	}
};
