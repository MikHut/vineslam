#pragma once

#include "vineslam_ros.hpp"

namespace vineslam
{
class LocalizationNode : public VineSLAM_ros
{
public:
  // Class constructor that
  // - Initialize the ROS node
  // - Define the publish and subscribe topics
  LocalizationNode();

  // Class destructor - saves the map to an output xml file
  ~LocalizationNode();

private:
  // Parameters loader
  void loadParameters(Parameters& params);

  // Runtime execution routines
  void init();
  void loop();
  void loopOnce();
  void process();

  // Routine to set the initial 6-DoF pose of the robot in relation with the previously built map
  void initializeOnMap();
  // Interactive marker callback functions
  void iMarkerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void iMenuCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  // Interactive marker for initialization variables
  interactive_markers::MenuHandler im_menu_handler_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> im_server_;

  // ROS subscribers
  rclcpp::Subscription<vineslam_msgs::msg::FeatureArray>::SharedPtr feature_subscriber_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr landmark_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscriber_;

  // ROS services
  rclcpp::Service<vineslam_ros::srv::SaveMap>::SharedPtr save_map_srv_;
};

}  // namespace vineslam