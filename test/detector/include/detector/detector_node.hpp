#pragma once

// Detector estimator
#include <localizer.hpp>
#include <mapper_2d.hpp>
#include <mapper_3d.hpp>

// ROS and iostream
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>
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

	// Performs the ros spin() method
	void run();
	// Saves the map into a .txt file format
	void saveMap();

private:
	// Left and depth image callback funcion
	void imageListener(const sensor_msgs::ImageConstPtr& msg_left,
	                   const sensor_msgs::ImageConstPtr& msg_depth);
	// Odometry callback function
	void odomListener(const nav_msgs::OdometryConstPtr& msg);

	// Object detection function
	// - Uses the EdgeTPU C++ library to perform inference
	std::vector<coral::DetectionCandidate>
	detect(const sensor_msgs::ImageConstPtr& img);

	// Computes the depth of an object using the ZED disparity image
	// - Calculates the median of all points to remove outliers
	double computeDepth(const sensor_msgs::Image& depth_img, const int& xmin,
	                    const int& ymin, const int& xmax, const int& ymax);

	// Converts a trunk observation into a bearing angle
	double columnToTheta(const int& col)
	{
		return (-h_fov / img_width) * (img_width / 2 - col);
	}

	// Publish map on rviz
	void publish2DMap(const std_msgs::Header& header, const Pose<double>& pose);
	// Publish a 3D pcl in sensor_msgs::PointCloud2 format
	void publish3DCloud();

#ifdef DEBUG
	// DEBUG method
	// - Publishes and image showing the detection bounding boxes
	void showBBoxes(const sensor_msgs::ImageConstPtr& msg, cv::Mat& bboxes,
	                const std::vector<coral::DetectionCandidate>& res);

	cv::Mat p_image;
	cv::Mat c_image;

	image_transport::Publisher l_img_publisher;
	image_transport::Publisher r_img_publisher;
#endif

	// ROS publishers
	ros::Publisher map_publisher;
	ros::Publisher particle_publisher;
	ros::Publisher odom_publisher;
	ros::Publisher pcl_publisher;
	ros::Publisher bbox_publisher;

	// Odometry variables
	Pose<double>       odom;
	Pose<double>       p_odom;
	nav_msgs::Odometry odom_;

	// Initialization boolean
	bool init;

	// Map of landmarks
	std::map<int, Landmark<double>> map_2d;

	// Classes object members
	Localizer*  localizer;
	Mapper2D*   mapper_2d;
	Mapper3D*   mapper_3d;
	Parameters* params;

	// EdgeTPU API members
	std::vector<int>                     input_tensor_shape;
	coral::DetectionEngine*              engine;
	std::unordered_map<int, std::string> labels;

	// Numberic input parameters
	double img_width;
	double h_fov;

	// Load all the parameters of the ROS node
	void loadParameters(const ros::NodeHandle& local_nh)
	{
		local_nh.getParam("/detector/image_left", (*params).image_left);
		local_nh.getParam("/detector/image_depth", (*params).image_depth);
		local_nh.getParam("/detector/odom_topic", (*params).odom_topic);
		local_nh.getParam("/detector/model_path", (*params).model);
		local_nh.getParam("/detector/labels_path", (*params).labels);
		local_nh.getParam("/detector/config_path", (*params).config);
	}

	// Initialize semantic information of feature to give
	// to the mapper class
	SemanticInfo fillSemanticInfo(const int& label)
	{
		SemanticInfo s;
		std::string  type;
		std::string  desc;
		int          ch;

		switch (label) {
		case 0:
			type = "Trunk";
			desc = "Vine trunk. A static landmark";
			ch   = 0;

			s = SemanticInfo(type, desc, ch);
			break;
		case 1:
			type = "Leaf";
			desc = "Leaf from a vine trunk. A dynamic landmark";
			ch   = 1;

			s = SemanticInfo(type, desc, ch);
			break;
		default:
			s = SemanticInfo("Trunk", "Vine trunk", 0);
		}

		return s;
	}
};
