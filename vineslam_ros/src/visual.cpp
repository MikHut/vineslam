#include "../include/vineslam_ros.hpp"

namespace vineslam
{

void vineslam_ros::publishGridMap(const std_msgs::Header& header)
{
  // Define ROS occupancy grid map
  nav_msgs::OccupancyGrid occ_map;
  occ_map.header          = header;
  occ_map.header.frame_id = "map";

  // Set the map metadata
  nav_msgs::MapMetaData metadata;
  metadata.origin.position.x    = occ_origin.x;
  metadata.origin.position.y    = occ_origin.y;
  metadata.origin.position.z    = occ_origin.z;
  metadata.origin.orientation.x = 0.;
  metadata.origin.orientation.y = 0.;
  metadata.origin.orientation.z = 0.;
  metadata.origin.orientation.w = 1.;
  metadata.resolution           = occ_resolution;
  metadata.width                = occ_width / occ_resolution;
  metadata.height               = occ_height / occ_resolution;
  occ_map.info                  = metadata;

  // Fill the occupancy grid map
  occ_map.data.resize(metadata.width * metadata.height);
  // - compute x and y bounds
  int xmin = static_cast<int>(occ_origin.x / occ_resolution);
  int xmax = static_cast<int>((float)xmin + occ_width / occ_resolution - 1);
  int ymin = static_cast<int>(occ_origin.y / occ_resolution);
  int ymax = static_cast<int>((float)ymin + occ_height / occ_resolution - 1);
  for (int i = xmin; i < xmax; i++) {
    for (int j = ymin; j < ymax; j++) {
      int8_t number_objs =
          (*grid_map)(i, j).landmarks.size() + (*grid_map)(i, j).landmarks.size();

      int m_i = i - static_cast<int>(occ_origin.x / occ_resolution);
      int m_j = j - static_cast<int>(occ_origin.y / occ_resolution);
      int idx = m_i + m_j * static_cast<int>((occ_width / occ_resolution));

      occ_map.data[idx] = number_objs * 10;
    }
  }

  // Publish the map
  mapOCC_publisher.publish(occ_map);
}

void vineslam_ros::publish2DMap(const std_msgs::Header&   header,
                                const pose&               pose,
                                const std::vector<float>& bearings,
                                const std::vector<float>& depths)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker      marker;
  visualization_msgs::MarkerArray ellipse_array;
  visualization_msgs::Marker      ellipse;

  // Define marker layout
  marker.ns                 = "/markers";
  marker.type               = visualization_msgs::Marker::CYLINDER;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.scale.x            = 0.1;
  marker.scale.y            = 0.1;
  marker.scale.z            = 0.3;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.r            = 0.0f;
  marker.color.g            = 0.0f;
  marker.color.b            = 1.0f;
  marker.color.a            = 1.0;
  marker.lifetime           = ros::Duration();

  // Define marker layout
  ellipse.ns                 = "/ellipses";
  ellipse.type               = visualization_msgs::Marker::CYLINDER;
  ellipse.action             = visualization_msgs::Marker::ADD;
  ellipse.scale.z            = 0.01f;
  ellipse.pose.orientation.x = 0.0f;
  ellipse.pose.orientation.y = 0.0f;
  ellipse.color.r            = 0.0f;
  ellipse.color.g            = 1.0f;
  ellipse.color.b            = 0.0f;
  ellipse.color.a            = 1.0f;
  ellipse.lifetime           = ros::Duration();

  // Publish markers
  int id = 1;
  for (auto& it : (*grid_map)) {
    for (const auto& m_sfeature : it.landmarks) {
      // Draw sfeature mean
      marker.id              = id;
      marker.header          = header;
      marker.header.frame_id = "map";
      marker.pose.position.x = m_sfeature.second.pos.x;
      marker.pose.position.y = m_sfeature.second.pos.y;
      marker.pose.position.z = m_sfeature.second.pos.z;

      marker_array.markers.push_back(marker);

      // Draw sfeature standard deviation
      tf2::Quaternion q;
      q.setRPY(0, 0, m_sfeature.second.gauss.theta);

      ellipse.id                 = id;
      ellipse.header             = header;
      ellipse.header.frame_id    = "map";
      ellipse.pose.position.x    = m_sfeature.second.pos.x;
      ellipse.pose.position.y    = m_sfeature.second.pos.y;
      ellipse.pose.position.z    = m_sfeature.second.pos.z;
      ellipse.scale.x            = 3 * m_sfeature.second.gauss.stdev.x;
      ellipse.scale.y            = 3 * m_sfeature.second.gauss.stdev.y;
      ellipse.pose.orientation.x = q.x();
      ellipse.pose.orientation.y = q.y();
      ellipse.pose.orientation.z = q.z();
      ellipse.pose.orientation.w = q.w();

      ellipse_array.markers.push_back(ellipse);

      id++;
    }
  }

  // Draw ellipse that characterizes particles distribution
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.dist.theta);

  ellipse.id                 = id;
  ellipse.header             = header;
  ellipse.header.frame_id    = "map";
  ellipse.pose.position.x    = pose.x;
  ellipse.pose.position.y    = pose.y;
  ellipse.pose.position.z    = pose.z;
  ellipse.scale.x            = 3 * pose.dist.stdev.x;
  ellipse.scale.y            = 3 * pose.dist.stdev.y;
  ellipse.pose.orientation.x = q.x();
  ellipse.pose.orientation.y = q.y();
  ellipse.pose.orientation.z = q.z();
  ellipse.pose.orientation.w = q.w();
  ellipse.color.r            = 0.0f;
  ellipse.color.g            = 0.0f;
  ellipse.color.b            = 1.0f;
  ellipse.color.a            = 1.0f;
  ellipse_array.markers.push_back(ellipse);

  map2D_publisher.publish(marker_array);
  map2D_publisher.publish(ellipse_array);
}

void vineslam_ros::publish3DMap()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  for (const auto& it : *grid_map) {
    for (const auto& feature : it.surf_features) {
      pcl::PointXYZRGB m_pt(feature.r, feature.g, feature.b);
      m_pt.x = feature.pos.x;
      m_pt.y = feature.pos.y;
      m_pt.z = feature.pos.z;

      cloud_out->points.push_back(m_pt);
    }
  }

  cloud_out->header.frame_id = "map";
  map3D_publisher.publish(cloud_out);
}

void vineslam_ros::publish3DMap(const std::vector<ImageFeature>& features,
                                const ros::Publisher&            pub)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  for (const auto& feature : features) {
    pcl::PointXYZRGB m_pt(feature.r, feature.g, feature.b);
    m_pt.x = feature.pos.x;
    m_pt.y = feature.pos.y;
    m_pt.z = feature.pos.z;

    cloud_out->points.push_back(m_pt);
  }

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

}; // namespace vineslam
