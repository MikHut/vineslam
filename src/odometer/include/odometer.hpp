#pragma once

#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <landmark_processor.hpp>
#include <nav_msgs/Odometry.h>
#include <opencv2/features2d.hpp>
#include <particle_filter.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

class Odometer
{
public:
	Odometer();
  ~Odometer();
	void boxListener(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);
	void imageListener(const sensor_msgs::ImageConstPtr& msg);
	/* void poseListener(const nav_msgs::OdometryConstPtr& msg); */
	void poseListener(const geometry_msgs::PoseStampedConstPtr& msg);

	image_transport::Publisher grid_pub;
	image_transport::Publisher matches_pub;
	image_transport::Publisher particles_pub;

	ros::Publisher pcl_pub;

private:
	double h_fov;
	double v_fov;
	double cam_height;
	int    width;
	int    heigth;
	int    resolution;
	int    match_box;

	bool init;

	Pose<double> last_pose;

	std_msgs::Header                scan_header;
	nav_msgs::Odometry              odom;
	geometry_msgs::Pose             slam_pose;
	darknet_ros_msgs::BoundingBoxes bounding_boxes;

	cv::Mat last_grid;
	cv::Mat c_image;
	cv::Mat p_image;

	ParticleFilter*    pfilter;
	LandmarkProcessor* lprocessor;
	Parameters*        params;

	void loadParameters(const ros::NodeHandle& local_nh)
	{
		/* read launch file parameters */
		local_nh.getParam("/odometer/h_fov", (*params).h_fov);
		local_nh.getParam("/odometer/v_fov", (*params).v_fov);
		local_nh.getParam("/odometer/img_width", (*params).width);
		local_nh.getParam("/odometer/img_height", (*params).height);
		local_nh.getParam("/odometer/grid_resolution", (*params).resolution);
		local_nh.getParam("/odometer/match_box", (*params).match_box);
	}
};
