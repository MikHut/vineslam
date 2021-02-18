#pragma once

#include "vineslam_ros.hpp"

namespace vineslam
{
class SLAMNode : public VineSLAM_ros, rclcpp::Node
{
public:
  // Class constructor that
  // - Initialize the ROS node
  // - Define the publish and subscribe topics
  SLAMNode(int argc, char** argv);

  // Class destructor - saves the map to an output xml file
  ~SLAMNode();

private:
  // ROS subscribers
  rclcpp::Subscription<vineslam_msgs::msg::FeatureArray>::SharedPtr feature_subscriber_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr landmark_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
};

}  // namespace vineslam