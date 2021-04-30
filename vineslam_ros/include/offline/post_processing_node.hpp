#pragma once

// vineslam members
#include <vineslam/feature/semantic.hpp>
#include <vineslam/feature/visual.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/mapping/elevation_map.hpp>
#include <vineslam/mapping/post_processing.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Const.hpp>
#include <vineslam/map_io/map_writer.hpp>
#include <vineslam/map_io/map_parser.hpp>
#include <vineslam/map_io/elevation_map_writer.hpp>
#include <vineslam/map_io/elevation_map_parser.hpp>
#include <vineslam/params.hpp>

// std
#include <iostream>
#include <string>
#include <chrono>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace vineslam
{
class MapPostProcessingNode : public rclcpp::Node
{
public:
  MapPostProcessingNode(const std::string& node);

private:
  // Publish routines
  void publishMaps();
  void publish3DMap();
  void publishElevationMap() const;

  // Objects
  OccupancyMap* grid_map_;
  ElevationMap* elevation_map_;
  MapPostProcessor* m_post_processor;
  Parameters params_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr elevation_map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map3D_corners_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map3D_planars_publisher_;
};
}  // namespace vineslam