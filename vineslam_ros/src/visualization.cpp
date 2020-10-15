#include "../include/vineslam_ros.hpp"

namespace vineslam
{

void VineSLAM_ros::publishGridMap(const std_msgs::Header& header) const
{
  // Define ROS occupancy grid map
  nav_msgs::OccupancyGrid occ_map;
  occ_map.header.stamp    = ros::Time::now();
  occ_map.header.frame_id = "map";

  // Set the map metadata
  nav_msgs::MapMetaData metadata;
  metadata.origin.position.x    = params.gridmap_origin_x;
  metadata.origin.position.y    = params.gridmap_origin_y;
  metadata.origin.position.z    = 0;
  metadata.origin.orientation.x = 0.;
  metadata.origin.orientation.y = 0.;
  metadata.origin.orientation.z = 0.;
  metadata.origin.orientation.w = 1.;
  metadata.resolution           = params.gridmap_resolution;
  metadata.width                = params.gridmap_width / params.gridmap_resolution;
  metadata.height               = params.gridmap_height / params.gridmap_resolution;
  occ_map.info                  = metadata;

  // Fill the occupancy grid map
  occ_map.data.resize(metadata.width * metadata.height);
  // - compute x and y bounds
  int xmin = static_cast<int>(params.gridmap_origin_x / params.gridmap_resolution);
  int xmax = static_cast<int>((float)xmin +
                              params.gridmap_width / params.gridmap_resolution - 1);
  int ymin = static_cast<int>(params.gridmap_origin_y / params.gridmap_resolution);
  int ymax = static_cast<int>((float)ymin +
                              params.gridmap_height / params.gridmap_resolution - 1);
  for (int i = xmin; i < xmax; i++) {
    for (int j = ymin; j < ymax; j++) {
      int8_t number_objs = (*grid_map)(i, j).landmarks.size() +
                           (*grid_map)(i, j).surf_features.size() +
                           (*grid_map)(i, j).corner_features.size();

      int m_i =
          i - static_cast<int>(params.gridmap_origin_x / params.gridmap_resolution);
      int m_j =
          j - static_cast<int>(params.gridmap_origin_y / params.gridmap_resolution);
      int idx = m_i + m_j * static_cast<int>(
                                (params.gridmap_width / params.gridmap_resolution));

      occ_map.data[idx] = number_objs * 10;
    }
  }

  // Publish the map
  mapOCC_publisher.publish(occ_map);
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
  for (auto& it : (*grid_map)) {
    for (const auto& m_sfeature : it.landmarks) {
      // Draw sfeature mean
      marker.id              = id;
      marker.header.stamp    = ros::Time::now();
      marker.header.frame_id = "map";
      marker.pose.position.x = m_sfeature.second.pos.x;
      marker.pose.position.y = m_sfeature.second.pos.y;
      marker.pose.position.z = m_sfeature.second.pos.z;

      marker_array.markers.push_back(marker);

      // Draw sfeature standard deviation
      tf2::Quaternion q;
      q.setRPY(0, 0, m_sfeature.second.gauss.theta);

      ellipse.id                 = id;
      ellipse.header.stamp       = ros::Time::now();
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
  ellipse.header.stamp       = ros::Time::now();
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

void VineSLAM_ros::publish3DMap() const
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (const auto& it : *grid_map) {
    for (const auto& feature : it.surf_features) {
      pcl::PointXYZRGB m_pt(feature.r, feature.g, feature.b);
      m_pt.x = feature.pos.x;
      m_pt.y = feature.pos.y;
      m_pt.z = feature.pos.z;

      feature_cloud->points.push_back(m_pt);
    }

    for (const auto& corner : it.corner_features) {
      pcl::PointXYZI m_pt(static_cast<float>(corner.which_cluster));
      m_pt.x = corner.pos.x;
      m_pt.y = corner.pos.y;
      m_pt.z = corner.pos.z;

      corner_cloud->points.push_back(m_pt);
    }
  }

  feature_cloud->header.frame_id = "map";
  corner_cloud->header.frame_id  = "map";
  plane_cloud->header.frame_id   = "map";
  map3D_features_publisher.publish(feature_cloud);
  map3D_corners_publisher.publish(corner_cloud);
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

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const std::vector<Corner>& corners,
                                const ros::Publisher&      pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointXYZI m_pt;

  for (const auto& corner : corners) {
    std::array<float, 9> robot_R{};
    robot_pose.toRotMatrix(robot_R);
    TF robot_tf(robot_R,
                std::array<float, 3>{robot_pose.x, robot_pose.y, robot_pose.z});

    m_pt.x = corner.pos.x * robot_tf.R[0] + corner.pos.y * robot_tf.R[1] +
             corner.pos.z * robot_tf.R[2] + robot_tf.t[0];
    m_pt.y = corner.pos.x * robot_tf.R[3] + corner.pos.y * robot_tf.R[4] +
             corner.pos.z * robot_tf.R[5] + robot_tf.t[1];
    m_pt.z = corner.pos.x * robot_tf.R[6] + corner.pos.y * robot_tf.R[7] +
             corner.pos.z * robot_tf.R[8] + robot_tf.t[2];

    m_pt.intensity = static_cast<float>(corner.which_cluster) * 10.0f;
    cloud_out->points.push_back(m_pt);
  }

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

void VineSLAM_ros::visDebug(const std::vector<Plane>& planes,
                            const Plane&              ground_plane)
{
  // Publish all poses for DEBUG
  // ----------------------------------------------------------------------------
  std::vector<pose> poses;
  (*localizer).getParticles(poses);
  geometry_msgs::PoseArray ros_poses;
  ros_poses.header.stamp    = ros::Time::now();
  ros_poses.header.frame_id = "map";
  for (const auto& pose : poses) {
    tf::Quaternion m_q;
    m_q.setRPY(pose.roll, pose.pitch, pose.yaw);
    m_q.normalize();

    geometry_msgs::Pose m_pose;
    m_pose.position.x    = pose.x;
    m_pose.position.y    = pose.y;
    m_pose.position.z    = pose.z;
    m_pose.orientation.x = m_q.x();
    m_pose.orientation.y = m_q.y();
    m_pose.orientation.z = m_q.z();
    m_pose.orientation.w = m_q.w();

    ros_poses.poses.push_back(m_pose);
  }
  poses_publisher.publish(ros_poses);

  // - Publish associations between corners
  visualization_msgs::MarkerArray markers;

  // - Publish lines for debug
  if (planes.size() == 2) {
    geometry_msgs::Point P1_a, P2_a, P1_b, P2_b;
    float                x_min_a = planes[0].points[0].x;
    float                x_max_a = planes[0].points[planes[0].points.size() - 1].x;
    float                x_min_b = planes[1].points[0].x;
    float                x_max_b = planes[1].points[planes[1].points.size() - 1].x;
    P1_a.x                       = x_min_a;
    P1_a.y = x_min_a * planes[0].regression.m + planes[0].regression.b;
    P1_a.z = 0;
    P2_a.x = x_max_a;
    P2_a.y = x_max_a * planes[0].regression.m + planes[0].regression.b;
    P2_a.z = 0;
    P1_b.x = x_min_b;
    P1_b.y = x_min_b * planes[1].regression.m + planes[1].regression.b;
    P1_b.z = 0;
    P2_b.x = x_max_b;
    P2_b.y = x_max_b * planes[1].regression.m + planes[1].regression.b;
    P2_b.z = 0;

    visualization_msgs::Marker marker_a;
    marker_a.header.frame_id = "base_link";
    marker_a.header.stamp    = ros::Time::now();
    marker_a.ns              = "line_a";
    marker_a.id              = 0;
    marker_a.type            = visualization_msgs::Marker::LINE_STRIP;
    marker_a.action          = visualization_msgs::Marker::ADD;
    marker_a.points.push_back(P1_a);
    marker_a.points.push_back(P2_a);
    marker_a.color.a = 1;
    marker_a.color.r = 1;
    marker_a.color.b = 0;
    marker_a.color.g = 0;
    marker_a.scale.x = 0.1;
    marker_a.scale.y = 0.1;
    marker_a.scale.z = 0.1;
    visualization_msgs::Marker marker_b;
    marker_b.header.frame_id = "base_link";
    marker_b.header.stamp    = ros::Time::now();
    marker_b.ns              = "line_b";
    marker_b.id              = 1;
    marker_b.type            = visualization_msgs::Marker::LINE_STRIP;
    marker_b.action          = visualization_msgs::Marker::ADD;
    marker_b.points.push_back(P1_b);
    marker_b.points.push_back(P2_b);
    marker_b.color.a = 1;
    marker_b.color.r = 0;
    marker_b.color.b = 1;
    marker_b.color.g = 0;
    marker_b.scale.x = 0.1;
    marker_b.scale.y = 0.1;
    marker_b.scale.z = 0.1;
    markers.markers.push_back(marker_a);
    markers.markers.push_back(marker_b);
  }

  debug_markers.publish(markers);
}

} // namespace vineslam