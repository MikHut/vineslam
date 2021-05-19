#pragma once

#include "../vineslam_ros.hpp"
#include <vineslam/matcher/icp.hpp>

namespace vineslam
{
class MappingNode : public VineSLAM_ros
{
public:
  // Class constructor that
  // - Initialize the ROS node
  // - Define the publish topics
  MappingNode();

private:
  // Parameters loader
  void loadParameters(Parameters& params);

  // Runtime execution routines
  void loop();
  void loopOnce(const std::vector<Planar>& points);

  // Iterative closest point member
  ICP<Planar>* icp_;

  // Iteration id
  uint32_t idx_;

  // ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map3D_publisher_;

  // ROS services
  rclcpp::Service<vineslam_ros::srv::SaveMap>::SharedPtr save_map_srv_;
};

}  // namespace vineslam
