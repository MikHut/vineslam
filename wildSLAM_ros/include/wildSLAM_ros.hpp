#pragma once

// wildSLAM members
#include <localizer.hpp>
#include <mapper_2d.hpp>
#include <mapper_3d.hpp>
#include <math/point3D.hpp>
#include <math/pose6D.hpp>

// std
#include <iostream>

// ROS
#include <cv_bridge/cv_bridge.h>
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
#include <geometry_msgs/PoseArray.h>

// OCTOMAP
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

namespace wildSLAM_ros
{
class SLAMNode
{
public:
	// Class constructor that
	// - Initializes the ROS node
	// - Defines the publish and subscribe topics
	SLAMNode(int argc, char** argv);

	// Callback function that subscribes a rgb image, a  disparity image,
	// and the bounding boxes that locate the objects on the image
	void callbackFct(const sensor_msgs::ImageConstPtr&            left_image,
	                 const sensor_msgs::ImageConstPtr&            depth_image,
	                 const vision_msgs::Detection2DArrayConstPtr& dets);
	// Odometry callback function
	void odomListener(const nav_msgs::OdometryConstPtr& msg);

private:
	// Publish map on rviz
	void publish2DMap(const std_msgs::Header& header, const pose6D& pose,
	                  const std::vector<float>& bearings,
	                  const std::vector<float>& depths);
	// Publish the 3D trunk map using a pcl
	void publish3DTrunkMap(const std_msgs::Header& header);
	// Computes the bearing depth of an object using the ZED disparity image
	// - Uses the point with minimum depth inside the bounding box
	void computeObsv(const sensor_msgs::Image& depth_img, const int& xmin,
	                 const int& ymin, const int& xmax, const int& ymax,
	                 float& depth, float& bearing);

	// Definitions of the ROS publishers
	ros::Publisher map2D_publisher;
	ros::Publisher map3D_publisher;
	ros::Publisher pose_publisher;
	ros::Publisher poses_publisher;

	// Classes object members
	Localizer* localizer;
	Mapper2D*  mapper2D;
	Mapper3D*  mapper3D;

	// Map of landmarks with
	// - the id of each landmark
	// - the position and semantic information of each landmark
	std::map<int, Landmark<float>> map2D;

	// Odometry pose variables
	pose6D odom;
	pose6D p_odom;

	// Numberic input parameters
	float h_fov;
	float img_width;
	float img_height;
	float cam_height;
	float fx;
	float fy;
	float cx;
	float cy;

	// Initialize flag
	bool init;
};
}; // namespace wildSLAM_ros
