#pragma once

#include "landmark_processor.hpp"
/* #include "mapper.hpp" */
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/features2d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt32MultiArray.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

/* edgetpu detection API */
#include <detection/engine.h>
#include <examples/label_utils.h>
#include <examples/model_utils.h>
#include <test_utils.h>

class Mapper
{

public:
	Mapper(int argc, char** argv);

	void run();
	void saveMap();

private:
	void imageListener(const sensor_msgs::ImageConstPtr& msg_left,
	                   const sensor_msgs::ImageConstPtr& msg_right,
                     const sensor_msgs::ImageConstPtr& msg_depth);

	std::vector<coral::DetectionCandidate>
	detect(const sensor_msgs::ImageConstPtr& img);
#ifdef DEBUG
	void showMatching(cv::Mat l_img, cv::Mat r_img);
	void showBBoxes(const sensor_msgs::ImageConstPtr& msg, cv::Mat& bboxes,
	                std::vector<coral::DetectionCandidate> res);

	cv::Mat p_image;
	cv::Mat c_image;

	image_transport::Publisher matches_publisher;
	image_transport::Publisher l_img_publisher;
	image_transport::Publisher r_img_publisher;
#endif

	bool init;

	/* Estimator*         estimator; */
	LandmarkProcessor* lprocessor;
	Parameters*        params;

	std::vector<Match<double>> matches;

	std::vector<int>                     input_tensor_shape;
	coral::DetectionEngine*              engine;
	std::unordered_map<int, std::string> labels;

	void loadParameters(const ros::NodeHandle& local_nh)
	{
		/* read launch file parameters */
		local_nh.getParam("/mapper/image_left", (*params).image_left);
		local_nh.getParam("/mapper/image_right", (*params).image_right);
		local_nh.getParam("/mapper/image_depth", (*params).image_depth);
		local_nh.getParam("/mapper/h_fov", (*params).h_fov);
		local_nh.getParam("/mapper/v_fov", (*params).v_fov);
		local_nh.getParam("/mapper/img_width", (*params).width);
		local_nh.getParam("/mapper/img_height", (*params).height);
		local_nh.getParam("/mapper/match_box", (*params).match_box);
		local_nh.getParam("/mapper/detector_th", (*params).min_score);
		local_nh.getParam("/mapper/model_path", (*params).model);
		local_nh.getParam("/mapper/labels_path", (*params).labels);
	}
};
