#pragma once

#include "../vineslam_ros.hpp"

// Mesh converter
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

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

  // Map register
  void registerPoints(Pose robot_pose, const std::vector<Planar>& points, OccupancyMap& grid_map);

  // Convert point cloud to mesh
  void cloudToMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh& mesh);

  // Visualization
  void meshToShapeMsg(const pcl::PolygonMesh& in, shape_msgs::msg::Mesh& mesh);
  void meshToMarkerMsg(const pcl::PolygonMesh& in, visualization_msgs::msg::Marker& marker);

  // Iteration id
  uint32_t idx_;

  // ROS publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr poses_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map3D_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_publisher_;

  // ROS services
  rclcpp::Service<vineslam_ros::srv::SaveMap>::SharedPtr save_map_srv_;

  // SLAM map information
  std::map<int, SemanticFeature> semantic_features_;

  // Sensors to base_link tfs
  Tf laser_to_base_tf_;
};

}  // namespace vineslam
