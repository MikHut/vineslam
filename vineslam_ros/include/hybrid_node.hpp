#pragma once

#include "vineslam_ros.hpp"
#include <vineslam/matcher/icp.hpp>

namespace vineslam
{
class HybridNode : public VineSLAM_ros
{
public:
  // Class constructor that
  // - Initialize the ROS node
  // - Define the publish and subscribe topics
  HybridNode();

  // Class destructor - saves the map to an output xml file
  ~HybridNode();

private:
  // Parameters loader
  void loadParameters(Parameters& params);

  // Runtime execution routines
  void init();
  void loop();
  void loopOnce();
  void process();

  // Thread to publish the tfs exported by the localization node
  // NOTE: We perform this process in a thread since we both need to do it on runtime and when setting the initial pose
  //       In the second case, if not in a thread, the tfs are only published when the used sends some feedback through
  //       the interactive marker.
  void broadcastTfs();

  // Routine to set the initial 6-DoF pose of the robot in relation with the previously built map
  void initializeOnMap();
  // Interactive marker callback functions
  void iMarkerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void iMenuCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  // Publish unoccupied zone
  void publishUnoccupiedZone(const std::vector<Point>& rectangle);

  // Interactive marker for initialization variables
  interactive_markers::MenuHandler im_menu_handler_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> im_server_;

  // ROS subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_subscriber_;

  // ROS publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr unoccupied_zone_publisher_;

  // ROS services
  rclcpp::Service<vineslam_ros::srv::SaveMap>::SharedPtr save_map_srv_;
};

}  // namespace vineslam
