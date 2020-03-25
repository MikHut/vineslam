#pragma once

// wildSLAM members
#include <localizer.hpp>
#include <mapper_2d.hpp>
#include <mapper_3d.hpp>

// std
#include <iostream>

// ROS
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

namespace wildSLAM_ros
{
class SLAMNode
{
public:
	// Class constructor that
	// - Initializes the ROS node
	// - Defines the publish and subscribe topics
	SLAMNode(int argc, char** argv);

	// Callback function that subscribes both a disparity image
	// and the bounding boxes that locate the objects on the image
	void callbackFct(const sensor_msgs::ImageConstPtr&            depth_image,
	                 const vision_msgs::Detection2DArrayConstPtr& dets);
	// Odometry callback function
	void odomListener(const nav_msgs::OdometryConstPtr& msg);

private:
	// Publish map on rviz
	void publish2DMap(const std_msgs::Header& header, const Pose<double>& pose);
	// Publish the 3D raw map using a pcl 
	void publish3DRawMap(const std_msgs::Header& header);
	// Publish the 3D trunk map using a pcl 
	void publish3DTrunkMap(const std_msgs::Header& header);
	// Computes the depth of an object using the ZED disparity image
	// - Calculates the median of all points to remove outliers
	double computeDepth(const sensor_msgs::Image& depth_img, const int& xmin,
	                    const int& ymin, const int& xmax, const int& ymax);

	// Definitions of the ROS publishers
	ros::Publisher map2D_publisher;
	ros::Publisher map3D_raw_publisher;
	ros::Publisher map3D_trunk_publisher;
	ros::Publisher particle_publisher;

	// Classes object members
	Localizer* localizer;
	Mapper2D*  mapper2D;
	Mapper3D*  mapper3D;

	// Map of landmarks with
	// - the id of each landmark
	// - the position and semantic information of each landmark
	std::map<int, Landmark<double>> map2D;

	// Odometry pose variables
	Pose<double> odom;
	Pose<double> p_odom;

	// Numberic input parameters
	double img_width;
	double h_fov;

	// Initialize flag
	bool init;
};
}; // namespace wildSLAM_ros
