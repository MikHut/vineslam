#pragma once

// wildSLAM members
#include <feature.hpp>
#include <localizer.hpp>
#include <occupancy_map.hpp>
#include <mapper_2d.hpp>
#include <mapper_3d.hpp>
#include <math/point.hpp>
#include <math/pose.hpp>

// std
#include <iostream>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <yaml-cpp/yaml.h>

#define DEBUG 1

namespace wildSLAM
{
class wildSLAM_ros
{
public:
  // Class constructor that
  // - Initializes the ROS node
  // - Defines the publish and subscribe topics
  wildSLAM_ros(int argc, char** argv);

  // Callback function that subscribes a rgb image, a  disparity image,
  // and the bounding boxes that locate the objects on the image
  void callbackFct(const sensor_msgs::ImageConstPtr&            left_image,
                   const sensor_msgs::ImageConstPtr&            depth_image,
                   const vision_msgs::Detection2DArrayConstPtr& dets);
  // Odometry callback function
  void odomListener(const nav_msgs::OdometryConstPtr& msg);

private:
  // Publish map on rviz
  void publish2DMap(const std_msgs::Header&   header,
                    const pose&               pose,
                    const std::vector<float>& bearings,
                    const std::vector<float>& depths);
  // Publish the 3D trunk map using a the grid map
  void publish3DMap();
  // Publish the 3D trunk map using an array of features
  void publish3DMap(const std::vector<Feature>& features, const ros::Publisher& pub);
  // Publish the grid map that contains all the maps
  void publishGridMap(const std_msgs::Header& header);

  // Computes the bearing depth of an object using the ZED disparity image
  // - Uses the point with minimum depth inside the bounding box
  void computeObsv(const sensor_msgs::Image& depth_img,
                   const int&                xmin,
                   const int&                ymin,
                   const int&                xmax,
                   const int&                ymax,
                   float&                    depth,
                   float&                    bearing) const;

  // Definitions of the ROS publishers
  ros::Publisher mapOCC_publisher;
  ros::Publisher map2D_publisher;
  ros::Publisher map3D_publisher;
  ros::Publisher pose_publisher;
  ros::Publisher path_publisher;
  ros::Publisher poses_publisher;
#ifdef DEBUG
  ros::Publisher source_map_publisher;
  ros::Publisher aligned_map_publisher;
#endif

  // Classes object members
  Localizer*    localizer;
  OccupancyMap* grid_map;
  Mapper2D*     mapper2D;
  Mapper3D*     mapper3D;

  // Array of poses to store and publish the robot path
  std::vector<geometry_msgs::PoseStamped> path;

  // Motion variables
  pose odom;
  pose p_odom;
  pose p_pose;

  // Numberic input parameters
  // ------------------------
  // Camera info parameters
  float h_fov;
  float img_width;
  float img_height;
  float cam_height;
  float fx;
  float fy;
  float cx;
  float cy;
  // ------------------------
  // Grid map dimensions
  // NOTE: corners are in reference to the given origin
  point occ_origin;
  float occ_resolution;
  float occ_width;
  float occ_height;

  // Initialize flag
  bool init;
};
}; // namespace wildSLAM
