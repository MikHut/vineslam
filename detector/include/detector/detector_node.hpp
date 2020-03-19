#pragma once

// Detector estimator
#include <localizer.hpp>
#include <mapper_2d.hpp>

// GPS service stuff
#include <agrob_map_transform/GetPose.h>
#include <agrob_map_transform/SetDatum.h>

// ROS and iostream
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// Edgetpu detection API
#include <detection/engine.h>
#include <examples/label_utils.h>
#include <examples/model_utils.h>
#include <test_utils.h>


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
		return (-(*params).h_fov / (*params).width) * ((*params).width / 2 - col);
	}

	// Publish map on rviz
	void publishMap(const std_msgs::Header& header, const Pose<double>& pose);

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

	ros::Publisher     map_publisher;
	ros::Publisher     particle_publisher;
	ros::Publisher     odom_publisher;
	ros::ServiceClient polar2pose;

	bool init;

	std::map<int, Landmark<double>> map;

	Pose<double>        odom;
	Pose<double>        p_odom;
	nav_msgs::Odometry  odom_;

	Localizer*  localizer;
	Mapper2D*     mapper;
	Parameters* params;

	std::vector<int>                     input_tensor_shape;
	coral::DetectionEngine*              engine;
	std::unordered_map<int, std::string> labels;

	// Load all the parameters of the ROS node
	void loadParameters(const ros::NodeHandle& local_nh)
	{
		local_nh.getParam("/detector/focal_length", (*params).f_length);
		local_nh.getParam("/detector/baseline", (*params).baseline);
		local_nh.getParam("/detector/img_width", (*params).width);
		local_nh.getParam("/detector/img_height", (*params).height);
		local_nh.getParam("/detector/detector_th", (*params).min_score);
		local_nh.getParam("/detector/disp_error", (*params).delta_D);
		local_nh.getParam("/detector/h_fov", (*params).h_fov);
		local_nh.getParam("/detector/n_particles", (*params).n_particles);

		local_nh.getParam("/detector/image_left", (*params).image_left);
		local_nh.getParam("/detector/image_depth", (*params).image_depth);
		local_nh.getParam("/detector/odom_topic", (*params).odom_topic);
		local_nh.getParam("/detector/model_path", (*params).model);
		local_nh.getParam("/detector/labels_path", (*params).labels);
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
