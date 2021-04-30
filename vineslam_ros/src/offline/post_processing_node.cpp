#include "../include/offline/post_processing_node.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vineslam::MapPostProcessingNode>("post_processing_node"));
  rclcpp::shutdown();

  return 0;
}

namespace vineslam
{
MapPostProcessingNode::MapPostProcessingNode(const std::string& node) : Node(node)
{
  // -----------------------------------------------------------------------------
  // ----- ROS communications
  // -----------------------------------------------------------------------------
  map3D_corners_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/corners", 10);
  map3D_planars_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/planars", 10);
  elevation_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/elevationMap", 10);

  // -----------------------------------------------------------------------------
  // ----- Load parameters
  // -----------------------------------------------------------------------------

  std::string prefix = this->get_name();
  std::string param;

  // Load params
  param = prefix + ".grid_map_file";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params_.map_input_file_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".elevation_map_file";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params_.elevation_map_input_file_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }

  // -----------------------------------------------------------------------------
  // ----- Load maps
  // -----------------------------------------------------------------------------
  RCLCPP_INFO(this->get_logger(), "Loading maps...");
  ElevationMapParser elevation_map_parser(params_);
  MapParser map_parser(params_);
  if (!map_parser.parseHeader(&params_))
  {
    RCLCPP_ERROR(this->get_logger(), "Map input file not found.");
    return;
  }
  else
  {
    grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0), 1, 1);
    elevation_map_ = new ElevationMap(params_, Pose(0, 0, 0, 0, 0, 0));
  }
  RCLCPP_INFO(this->get_logger(), "The maps have been loaded.");

  // -----------------------------------------------------------------------------
  // ----- Declare post processor object
  // -----------------------------------------------------------------------------
  m_post_processor = new MapPostProcessor(grid_map_, elevation_map_);

  // -----------------------------------------------------------------------------
  // ----- Runtime threads
  // -----------------------------------------------------------------------------
  std::thread th0(&MapPostProcessingNode::publishMaps, this);
  th0.detach();
}

void MapPostProcessingNode::publishMaps()
{
  uint32_t mil_secs = static_cast<uint32_t>((1 / 3) * 1e3);
  while (rclcpp::ok())
  {
    // Publish 3D maps
    publish3DMap();
    publishElevationMap();

    // Impose loop frequency
    rclcpp::sleep_for(std::chrono::milliseconds(mil_secs));
  }
}

void MapPostProcessingNode::publishElevationMap() const
{
  visualization_msgs::msg::MarkerArray elevation_map_marker;
  visualization_msgs::msg::Marker cube;

  float min_height = grid_map_->origin_.z_;
  float max_height = grid_map_->origin_.z_ + grid_map_->height_;

  // Define marker layout
  cube.ns = "/elevation_cube";
  cube.header.stamp = rclcpp::Time();
  cube.header.frame_id = "world";
  cube.type = visualization_msgs::msg::Marker::CUBE;
  cube.action = visualization_msgs::msg::Marker::ADD;
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;
  cube.color.a = 1.0;
  cube.lifetime = rclcpp::Duration(970000000);

  // Compute map layer bounds
  float xmin = elevation_map_->origin_.x_;
  float xmax = xmin + elevation_map_->width_;
  float ymin = elevation_map_->origin_.y_;
  float ymax = ymin + elevation_map_->lenght_;
  for (float i = xmin; i < xmax - elevation_map_->resolution_;)
  {
    for (float j = ymin; j < ymax - elevation_map_->resolution_;)
    {
      float z = (*elevation_map_)(i, j);
      if (z == 0)
      {
        j += elevation_map_->resolution_;
        continue;
      }

      float r, g, b;
      float h = (static_cast<float>(1) -
                 std::min(std::max((std::fabs(z) - min_height) / (max_height - min_height), static_cast<float>(0)),
                          static_cast<float>(1)));
      vineslam::ElevationMap::color(h, r, g, b);

      cube.pose.position.x = i;
      cube.pose.position.y = j;
      cube.pose.position.z = z / 2;
      cube.scale.x = elevation_map_->resolution_;
      cube.scale.y = elevation_map_->resolution_;
      cube.scale.z = z;
      cube.color.r = r;
      cube.color.g = g;
      cube.color.b = b;
      cube.id = elevation_map_marker.markers.size();

      elevation_map_marker.markers.push_back(cube);

      j += grid_map_->resolution_;
    }
    i += grid_map_->resolution_;
  }

  elevation_map_publisher_->publish(elevation_map_marker);
}

void MapPostProcessingNode::publish3DMap()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  std::vector<Corner> corner_features = grid_map_->getCorners();
  std::vector<Planar> planar_features = grid_map_->getPlanars();

  for (const auto& corner : corner_features)
  {
    pcl::PointXYZI l_pt(static_cast<float>(corner.which_plane_));
    l_pt.x = corner.pos_.x_;
    l_pt.y = corner.pos_.y_;
    l_pt.z = corner.pos_.z_;

    corner_cloud->points.push_back(l_pt);
  }

  for (const auto& planar : planar_features)
  {
    pcl::PointXYZI l_pt(static_cast<float>(planar.which_plane_));
    l_pt.x = planar.pos_.x_;
    l_pt.y = planar.pos_.y_;
    l_pt.z = planar.pos_.z_;

    planar_cloud->points.push_back(l_pt);
  }

  corner_cloud->header.frame_id = params_.world_frame_id_;
  sensor_msgs::msg::PointCloud2 corner_cloud2;
  pcl::toROSMsg(*corner_cloud, corner_cloud2);
  map3D_corners_publisher_->publish(corner_cloud2);

  planar_cloud->header.frame_id = params_.world_frame_id_;
  sensor_msgs::msg::PointCloud2 planar_cloud2;
  pcl::toROSMsg(*planar_cloud, planar_cloud2);
  map3D_planars_publisher_->publish(planar_cloud2);
}

}  // namespace vineslam