#pragma once

#include "mapper.hpp"
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/features2d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt32MultiArray.h>
#include <tf/transform_broadcaster.h>
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
	// Class contructor
	// - Initializes the ROS node
	// - Performs all the subscriptions/publications
	// - Loads the NN model and labels file
	Mapper(int argc, char** argv);

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
	void publishMap(const std_msgs::Header& header);

#ifdef DEBUG
	// DEBUG method
	// - Publishes an image showing the matching of trunks between two images
	void showMatching(cv::Mat l_img, cv::Mat r_img);
	// DEBUG method
	// - Publishes and image showing the detection bounding boxes
	void showBBoxes(const sensor_msgs::ImageConstPtr& msg, cv::Mat& bboxes,
	                const std::vector<coral::DetectionCandidate>& res);

	cv::Mat p_image;
	cv::Mat c_image;

	image_transport::Publisher matches_publisher;
	image_transport::Publisher l_img_publisher;
	image_transport::Publisher r_img_publisher;
#endif

	ros::Publisher map_publisher;

	bool init;

	std::vector<Match<double>>      matches;
	std::map<int, Landmark<double>> map;

	Pose<double>       odom;
	nav_msgs::Odometry odom_;

	Estimator*  estimator;
	Parameters* params;

	std::vector<int>                     input_tensor_shape;
	coral::DetectionEngine*              engine;
	std::unordered_map<int, std::string> labels;

	// Load all the parameters of the ROS node
	void loadParameters(const ros::NodeHandle& local_nh)
	{
		local_nh.getParam("/mapper/image_left", (*params).image_left);
		local_nh.getParam("/mapper/image_right", (*params).image_right);
		local_nh.getParam("/mapper/image_depth", (*params).image_depth);
		local_nh.getParam("/mapper/odom_topic", (*params).odom_topic);
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
