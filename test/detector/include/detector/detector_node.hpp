#pragma once

#include <math/point3D.hpp>

// ROS and iostream
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>

// Edgetpu detection API
#include <detection/engine.h>
#include <examples/label_utils.h>
#include <examples/model_utils.h>
#include <test_utils.h>

// Struct that encodes ROS input parameters
struct Parameters
{
	std::string image_left;  // left image ROS topic
	std::string image_depth; // depth image ROS topic
	std::string odom_topic;  // odometry ROS topic
	std::string model;       // tflite model path
	std::string labels;      // detection labels path
	std::string config;      // config file path

	Parameters()
	{
		model       = "";
		labels      = "";
		config      = "";
		image_left  = "";
		image_depth = "";
		odom_topic  = "";
	}
};

class Detector
{

public:
	// Class contructor
	// - Initializes the ROS node
	// - Performs all the subscriptions/publications
	// - Loads the NN model and labels file
	Detector(int argc, char** argv);

private:
	// Left and depth image callback funcion
	void imageListener(const sensor_msgs::ImageConstPtr& msg_left,
	                   const sensor_msgs::ImageConstPtr& msg_depth);

	// Object detection function
	// - Uses the EdgeTPU C++ library to perform inference
	std::vector<coral::DetectionCandidate>
	detect(const sensor_msgs::ImageConstPtr& img);

	// DEBUG method
	// - Publishes and image showing the detection bounding boxes
	void showBBoxes(const sensor_msgs::ImageConstPtr&             left_image,
	                const sensor_msgs::ImageConstPtr&             depth_image,
	                cv::Mat&                                      bboxes,
	                const std::vector<coral::DetectionCandidate>& res);

	// DEGUB method
	// - Computes the depth of a trunk giving a bounding box and a depth
	// image
	point3D computeDepth(const sensor_msgs::Image& depth_img, const int& xmin,
	                     const int& ymin, const int& xmax, const int& ymax);

	// ROS publishers
	ros::Publisher             bbox_publisher;
	image_transport::Publisher l_img_publisher;

	// EdgeTPU API members
	std::vector<int>                     input_tensor_shape;
	coral::DetectionEngine*              engine;
	std::unordered_map<int, std::string> labels;

	// Load all the parameters of the ROS node
	Parameters* params;
	void        loadParameters(const ros::NodeHandle& local_nh)
	{
		local_nh.getParam("/detector/image_left", (*params).image_left);
		local_nh.getParam("/detector/image_depth", (*params).image_depth);
		local_nh.getParam("/detector/odom_topic", (*params).odom_topic);
		local_nh.getParam("/detector/model_path", (*params).model);
		local_nh.getParam("/detector/labels_path", (*params).labels);
		local_nh.getParam("/detector/config_path", (*params).config);
	}
};
