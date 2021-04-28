#pragma once

#include "vineslam_ros.hpp"
#include "../include/convertions.hpp"

namespace vineslam
{
class SLAMNode : public VineSLAM_ros
{
public:
  // Class constructor that
  // - Initialize the ROS node
  // - Define the publish and subscribe topics
  SLAMNode();

  // Class destructor - saves the map to an output xml file
  ~SLAMNode();

private:
  // Parameters loader
  void loadParameters(Parameters& params);

  // Runtime execution routines
  void init();
  void loop();
  void loopOnce();
  void process();

  // GNSS heading estimation
  void getGNSSHeading();

  // GNSS estimation variables
  int datum_autocorrection_stage;
  int32_t global_counter;
  double datum_orientation[360][4];

  // ROS subscribers
  rclcpp::Subscription<vineslam_msgs::msg::FeatureArray>::SharedPtr feature_subscriber_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr landmark_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
  //  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscriber_;

  // ROS services
  rclcpp::Service<vineslam_ros::srv::SaveMap>::SharedPtr save_map_srv_;
};

}  // namespace vineslam