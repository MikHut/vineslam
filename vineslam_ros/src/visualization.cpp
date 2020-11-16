#include "../include/vineslam_ros.hpp"

namespace vineslam
{

void VineSLAM_ros::publishGridMap(const std_msgs::Header& header) const
{
  // -------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------
  // ----
  // ---- WARNING : This visualization function is very slow. Use only for debug (!)
  // ----
  // -------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------

  // Define ROS occupancy grid map
  visualization_msgs::MarkerArray occ_map;

  int idx = 0;
  for (auto layer : *grid_map) {
    // Compute map layer bounds
    float xmin = layer.second.origin.x;
    float xmax = xmin + layer.second.width;
    float ymin = layer.second.origin.y;
    float ymax = xmin + layer.second.lenght;
    float zmin = static_cast<float>(layer.first) * grid_map->resolution_z +
                 grid_map->origin.z;
    for (float i = xmin; i < xmax - grid_map->resolution;) {
      for (float j = ymin; j < ymax - grid_map->resolution;) {
        int8_t number_objs = layer.second(i, j).landmarks.size() +
                             layer.second(i, j).surf_features.size() +
                             layer.second(i, j).corner_features.size();

        if (number_objs == 0) {
          j += grid_map->resolution;
          continue;
        }

        visualization_msgs::Marker cube_cell;
        cube_cell.header.frame_id = "odom";
        cube_cell.header.stamp    = ros::Time::now();
        cube_cell.id              = idx;
        cube_cell.type            = visualization_msgs::Marker::CUBE;
        cube_cell.action          = visualization_msgs::Marker::ADD;
        cube_cell.color.a         = 0.7;
        cube_cell.color.r         = 0;
        cube_cell.color.b =
            1 - (static_cast<float>(1) / static_cast<float>(number_objs));
        cube_cell.color.g            = 0;
        cube_cell.pose.position.x    = i;
        cube_cell.pose.position.y    = j;
        cube_cell.pose.position.z    = zmin;
        cube_cell.pose.orientation.x = 0;
        cube_cell.pose.orientation.y = 0;
        cube_cell.pose.orientation.z = 0;
        cube_cell.pose.orientation.w = 1;
        cube_cell.scale.x            = grid_map->resolution;
        cube_cell.scale.y            = grid_map->resolution;
        cube_cell.scale.z            = grid_map->resolution_z;

        occ_map.markers.push_back(cube_cell);

        idx++;
        j += grid_map->resolution;
      }
      i += grid_map->resolution;
    }
  }

  // Publish the map
  grid_map_publisher.publish(occ_map);
}

void VineSLAM_ros::publish2DMap(const std_msgs::Header&   header,
                                const pose&               pose,
                                const std::vector<float>& bearings,
                                const std::vector<float>& depths) const
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
  for (auto& it : (*grid_map)(0)) {
    for (const auto& m_sfeature : it.landmarks) {
      // Draw sfeature mean
      marker.id              = id;
      marker.header.stamp    = ros::Time::now();
      marker.header.frame_id = "odom";
      marker.pose.position.x = m_sfeature.second.pos.x;
      marker.pose.position.y = m_sfeature.second.pos.y;
      marker.pose.position.z = m_sfeature.second.pos.z;

      marker_array.markers.push_back(marker);

      // Draw sfeature standard deviation
      tf2::Quaternion q;
      q.setRPY(0, 0, m_sfeature.second.gauss.theta);

      ellipse.id                 = id;
      ellipse.header.stamp       = ros::Time::now();
      ellipse.header.frame_id    = "odom";
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
  ellipse.header.stamp       = ros::Time::now();
  ellipse.header.frame_id    = "odom";
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

void VineSLAM_ros::publish3DMap() const
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr planar_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (auto& it : *grid_map) {
    for (const auto& cell : it.second) {
      for (const auto& feature : cell.surf_features) {
        pcl::PointXYZRGB m_pt(feature.r, feature.g, feature.b);
        m_pt.x = feature.pos.x;
        m_pt.y = feature.pos.y;
        m_pt.z = feature.pos.z;

        feature_cloud->points.push_back(m_pt);
      }

      for (const auto& corner : cell.corner_features) {
        pcl::PointXYZI m_pt(static_cast<float>(corner.which_cluster));
        m_pt.x = corner.pos.x;
        m_pt.y = corner.pos.y;
        m_pt.z = corner.pos.z;

        corner_cloud->points.push_back(m_pt);
      }

      for (const auto& planar : cell.planar_features) {
        pcl::PointXYZI m_pt(static_cast<float>(planar.which_cluster));
        m_pt.x = planar.pos.x;
        m_pt.y = planar.pos.y;
        m_pt.z = planar.pos.z;

        planar_cloud->points.push_back(m_pt);
      }
    }
  }

  feature_cloud->header.frame_id = "odom";
  corner_cloud->header.frame_id  = "odom";
  planar_cloud->header.frame_id  = "odom";
  map3D_features_publisher.publish(feature_cloud);
  map3D_corners_publisher.publish(corner_cloud);
  map3D_planars_publisher.publish(planar_cloud);
}

void VineSLAM_ros::publish3DMap(const std::vector<Plane>& planes,
                                const ros::Publisher&     pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  std::array<float, 9> robot_R{};
  robot_pose.toRotMatrix(robot_R);
  TF robot_tf(robot_R,
              std::array<float, 3>{robot_pose.x, robot_pose.y, robot_pose.z});

  int i = 0;
  for (const auto& plane : planes) {
    for (const auto& pt : plane.points) {
      point m_pt = pt * robot_tf;

      pcl::PointXYZI pcl_pt(i);
      pcl_pt.x = m_pt.x;
      pcl_pt.y = m_pt.y;
      pcl_pt.z = m_pt.z;

      cloud_out->points.push_back(pcl_pt);
    }
    i++;
  }

  cloud_out->header.frame_id = "odom";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const pose&               r_pose,
                                const std::vector<Plane>& planes,
                                const ros::Publisher&     pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  TF robot_tf(robot_R, std::array<float, 3>{r_pose.x, r_pose.y, r_pose.z});

  int i = 0;
  for (const auto& plane : planes) {
    for (const auto& pt : plane.points) {
      point m_pt = pt * robot_tf;

      pcl::PointXYZI pcl_pt(i);
      pcl_pt.x = m_pt.x;
      pcl_pt.y = m_pt.y;
      pcl_pt.z = m_pt.z;

      cloud_out->points.push_back(pcl_pt);
    }
    i++;
  }

  cloud_out->header.frame_id = "odom";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const std::vector<Corner>& corners,
                                const ros::Publisher&      pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  std::array<float, 9> robot_R{};
  robot_pose.toRotMatrix(robot_R);
  TF robot_tf(robot_R,
              std::array<float, 3>{robot_pose.x, robot_pose.y, robot_pose.z});

  for (const auto& corner : corners) {
    point m_pt = corner.pos * robot_tf;

    pcl::PointXYZI pcl_pt;
    pcl_pt.x = m_pt.x;
    pcl_pt.y = m_pt.y;
    pcl_pt.z = m_pt.z;

    pcl_pt.intensity = static_cast<float>(corner.which_cluster) * 10.0f;
    cloud_out->points.push_back(pcl_pt);
  }

  cloud_out->header.frame_id = "odom";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const pose&                r_pose,
                                const std::vector<Corner>& corners,
                                const ros::Publisher&      pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  TF robot_tf(robot_R, std::array<float, 3>{r_pose.x, r_pose.y, r_pose.z});

  for (const auto& corner : corners) {
    point m_pt = corner.pos * robot_tf;

    pcl::PointXYZI pcl_pt;
    pcl_pt.x = m_pt.x;
    pcl_pt.y = m_pt.y;
    pcl_pt.z = m_pt.z;

    pcl_pt.intensity = static_cast<float>(corner.which_cluster) * 10.0f;
    cloud_out->points.push_back(pcl_pt);
  }

  cloud_out->header.frame_id = "odom";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const std::vector<Planar>& planars,
                                const ros::Publisher&      pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  std::array<float, 9> robot_R{};
  robot_pose.toRotMatrix(robot_R);
  TF robot_tf(robot_R,
              std::array<float, 3>{robot_pose.x, robot_pose.y, robot_pose.z});

  for (const auto& planar_feature : planars) {
    point m_pt = planar_feature.pos * robot_tf;

    pcl::PointXYZI pcl_pt;
    pcl_pt.x = m_pt.x;
    pcl_pt.y = m_pt.y;
    pcl_pt.z = m_pt.z;

    pcl_pt.intensity = static_cast<float>(planar_feature.which_cluster) * 10.0f;
    cloud_out->points.push_back(pcl_pt);
  }

  cloud_out->header.frame_id = "odom";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const pose&                r_pose,
                                const std::vector<Planar>& planars,
                                const ros::Publisher&      pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  TF robot_tf(robot_R, std::array<float, 3>{r_pose.x, r_pose.y, r_pose.z});

  for (const auto& planar_feature : planars) {
    point m_pt = planar_feature.pos * robot_tf;

    pcl::PointXYZI pcl_pt;
    pcl_pt.x = m_pt.x;
    pcl_pt.y = m_pt.y;
    pcl_pt.z = m_pt.z;

    pcl_pt.intensity = static_cast<float>(planar_feature.which_cluster) * 10.0f;
    cloud_out->points.push_back(pcl_pt);
  }

  cloud_out->header.frame_id = "odom";
  pub.publish(cloud_out);
}

} // namespace vineslam