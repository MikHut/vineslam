#include "../include/vineslam_ros.hpp"

namespace vineslam
{
void VineSLAM_ros::publishDenseInfo(const float& rate)
{
  ros::Rate r(10);
  while (ros::ok())
  {
    // We publish the 3D map also before the initialization for visualization purposes
    publish3DMap();

    if (init_flag_)
      continue;

    // Publish the 2D map
    publish2DMap();
    // Publish 3D maps
    publishElevationMap();
    publishGridMapLimits();

    // Impose loop frequency
    r.sleep();
  }
}

void VineSLAM_ros::publishGridMapLimits() const
{
  visualization_msgs::MarkerArray marker_array;

  // Publish the map
  visualization_msgs::Marker grid_map_cube;
  grid_map_cube.header.frame_id = params_.world_frame_id_;
  grid_map_cube.header.stamp = ros::Time::now();
  grid_map_cube.id = 0;
  grid_map_cube.type = visualization_msgs::Marker::CUBE;
  grid_map_cube.action = visualization_msgs::Marker::ADD;
  grid_map_cube.color.a = 0.7;
  grid_map_cube.color.r = 0;
  grid_map_cube.color.g = 1;
  grid_map_cube.color.b = 0;
  grid_map_cube.pose.position.x = grid_map_->origin_.x_ + grid_map_->width_ / 2;
  grid_map_cube.pose.position.y = grid_map_->origin_.y_ + grid_map_->lenght_ / 2;
  grid_map_cube.pose.position.z = grid_map_->origin_.z_ + grid_map_->height_ / 2;
  grid_map_cube.pose.orientation.x = 0;
  grid_map_cube.pose.orientation.y = 0;
  grid_map_cube.pose.orientation.z = 0;
  grid_map_cube.pose.orientation.w = 1;
  grid_map_cube.scale.x = grid_map_->width_;
  grid_map_cube.scale.y = grid_map_->lenght_;
  grid_map_cube.scale.z = grid_map_->height_;

  marker_array.markers.push_back(grid_map_cube);
  grid_map_publisher_.publish(marker_array);
}

void VineSLAM_ros::publishRobotBox(const Pose& robot_pose) const
{
  // Robot pose orientation to quaternion
  tf2::Quaternion q;
  q.setRPY(robot_pose.R_, robot_pose.P_, robot_pose.Y_);

  // Set robot original box corners
  visualization_msgs::Marker robot_cube;
  robot_cube.header.frame_id = params_.world_frame_id_;
  robot_cube.header.stamp = ros::Time::now();
  robot_cube.id = 0;
  robot_cube.type = visualization_msgs::Marker::CUBE;
  robot_cube.action = visualization_msgs::Marker::ADD;
  robot_cube.color.a = 0.7;
  robot_cube.color.r = 0;
  robot_cube.color.g = 0;
  robot_cube.color.b = 1;
  robot_cube.scale.x = params_.robot_dim_x_;
  robot_cube.scale.y = params_.robot_dim_y_;
  robot_cube.scale.z = params_.robot_dim_z_;
  robot_cube.pose.position.x = robot_pose.x_;
  robot_cube.pose.position.y = robot_pose.y_;
  robot_cube.pose.position.z = robot_pose.z_ + params_.robot_dim_z_ / 2;
  robot_cube.pose.orientation.x = q.getX();
  robot_cube.pose.orientation.y = q.getY();
  robot_cube.pose.orientation.z = q.getZ();
  robot_cube.pose.orientation.w = q.getW();

  robot_box_publisher_.publish(robot_cube);
}

void VineSLAM_ros::publish2DMap() const
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  visualization_msgs::Marker ellipse;

  // Define marker layout
  marker.ns = "/markers";
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.3;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  // Define marker layout
  ellipse.ns = "/ellipses";
  ellipse.type = visualization_msgs::Marker::CYLINDER;
  ellipse.action = visualization_msgs::Marker::ADD;
  ellipse.scale.z = 0.01f;
  ellipse.pose.orientation.x = 0.0f;
  ellipse.pose.orientation.y = 0.0f;
  ellipse.color.r = 0.0f;
  ellipse.color.g = 1.0f;
  ellipse.color.b = 0.0f;
  ellipse.color.a = 1.0f;

  std::map<int, SemanticFeature> l_landmarks = (*grid_map_)(0).getLandmarks();

  // Publish markers
  int id = 1;
  for (const auto& l_sfeature : l_landmarks)
  {
    // Draw sfeature mean
    marker.ns = "/marker";
    marker.id = id;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = params_.world_frame_id_;
    marker.pose.position.x = l_sfeature.second.pos_.x_;
    marker.pose.position.y = l_sfeature.second.pos_.y_;
    marker.pose.position.z = l_sfeature.second.pos_.z_;

    marker_array.markers.push_back(marker);

    // Draw sfeature standard deviation
    tf2::Quaternion q;
    q.setRPY(0, 0, l_sfeature.second.gauss_.theta_);

    ellipse.ns = "/ellipse";
    ellipse.id = id;
    ellipse.header.stamp = ros::Time::now();
    ellipse.header.frame_id = params_.world_frame_id_;
    ellipse.pose.position.x = l_sfeature.second.pos_.x_;
    ellipse.pose.position.y = l_sfeature.second.pos_.y_;
    ellipse.pose.position.z = l_sfeature.second.pos_.z_;
    ellipse.scale.x = 3 * l_sfeature.second.gauss_.stdev_.x_;
    ellipse.scale.y = 3 * l_sfeature.second.gauss_.stdev_.y_;
    ellipse.pose.orientation.x = q.x();
    ellipse.pose.orientation.y = q.y();
    ellipse.pose.orientation.z = q.z();
    ellipse.pose.orientation.w = q.w();

    marker_array.markers.push_back(ellipse);

    id++;
  }

  map2D_publisher_.publish(marker_array);
}

void VineSLAM_ros::publishElevationMap() const
{
  visualization_msgs::MarkerArray elevation_map_marker;
  visualization_msgs::Marker cube;

  float min_height = grid_map_->origin_.z_;
  float max_height = grid_map_->origin_.z_ + grid_map_->height_;

  // Define marker layout
  cube.ns = "/elevation_cube";
  cube.header.stamp = ros::Time::now();
  cube.header.frame_id = params_.world_frame_id_;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;
  cube.color.a = 1.0;

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

  elevation_map_publisher_.publish(elevation_map_marker);
}

void VineSLAM_ros::publish3DMap()
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

  publish3DMap(Pose(0, 0, 0, 0, 0, 0), grid_map_->planes_, map3D_planes_publisher_);

  corner_cloud->header.frame_id = params_.world_frame_id_;
  sensor_msgs::PointCloud2 corner_cloud2;
  pcl::toROSMsg(*corner_cloud, corner_cloud2);
  map3D_corners_publisher_.publish(corner_cloud2);

  planar_cloud->header.frame_id = params_.world_frame_id_;
  sensor_msgs::PointCloud2 planar_cloud2;
  pcl::toROSMsg(*planar_cloud, planar_cloud2);
  map3D_planars_publisher_.publish(planar_cloud2);
}

void VineSLAM_ros::publish3DMap(const std::vector<Plane>& planes,
                                const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker viz_pts;

  // Define marker layout
  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::Marker::POINTS;
  viz_pts.action = visualization_msgs::Marker::ADD;
  viz_pts.scale.x = 0.1;
  viz_pts.scale.y = 0.1;
  viz_pts.header.frame_id = params_.world_frame_id_;

  std::array<float, 9> robot_R{};
  robot_pose_.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ robot_pose_.x_, robot_pose_.y_, robot_pose_.z_ });

  float i = 0;
  for (const auto& plane : planes)
  {
    for (const auto& pt : plane.points_)
    {
      Point l_pt = pt * robot_tf;

      geometry_msgs::Point viz_pt;
      viz_pts.color.r = 0.0f;
      viz_pts.color.g = 0.0f;
      viz_pts.color.b = i / 3;
      viz_pts.color.a = 1.0;
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;

      viz_pts.points.push_back(viz_pt);
    }
    i += 1;
  }

  marker_array.markers.push_back(viz_pts);
  pub.publish(marker_array);
}

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<Plane>& planes,
                                const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker viz_pts;

  // Define marker layout
  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::Marker::POINTS;
  viz_pts.action = visualization_msgs::Marker::ADD;
  viz_pts.scale.x = 0.1;
  viz_pts.scale.y = 0.1;
  viz_pts.header.frame_id = params_.world_frame_id_;

  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ r_pose.x_, r_pose.y_, r_pose.z_ });

  float i = 0;
  for (const auto& plane : planes)
  {
    for (const auto& pt : plane.points_)
    {
      Point l_pt = pt * robot_tf;

      geometry_msgs::Point viz_pt;
      viz_pts.color.r = 0.0f;
      viz_pts.color.g = 0.0f;
      viz_pts.color.b = i * 10;
      viz_pts.color.a = 1.0;
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;

      viz_pts.points.push_back(viz_pt);
    }
    i += 1;
  }

  marker_array.markers.push_back(viz_pts);
  pub.publish(marker_array);
}

void VineSLAM_ros::publish3DMap(const std::vector<SemiPlane>& planes,
                                const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker viz_pts;
  visualization_msgs::Marker viz_line, viz_line1;
  visualization_msgs::Marker viz_normal;

  // Define markers layout

  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::Marker::POINTS;
  viz_pts.action = visualization_msgs::Marker::ADD;
  viz_pts.scale.x = 0.1;
  viz_pts.header.frame_id = params_.world_frame_id_;

  viz_line.ns = "/plane_extremas";
  viz_line.type = visualization_msgs::Marker::LINE_STRIP;
  viz_line.action = visualization_msgs::Marker::ADD;
  viz_line.scale.x = 0.1;
  viz_line.color.r = 0.0;
  viz_line.color.g = 0.0;
  viz_line.color.b = 1.0;
  viz_line.color.a = 1.0;
  viz_line.header.frame_id = params_.world_frame_id_;

  viz_normal.ns = "/plane_normal";
  viz_normal.type = visualization_msgs::Marker::LINE_STRIP;
  viz_normal.action = visualization_msgs::Marker::ADD;
  viz_normal.color.a = 1;
  viz_normal.color.r = 1;
  viz_normal.color.b = 0;
  viz_normal.color.g = 0;
  viz_normal.scale.x = 0.1;
  viz_normal.header.frame_id = params_.world_frame_id_;

  std::array<float, 9> robot_R{};
  robot_pose_.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ robot_pose_.x_, robot_pose_.y_, robot_pose_.z_ });

  float i = 0;
  float l = 0;
  for (const auto& plane : planes)
  {
    for (const auto& pt : plane.points_)
    {
      Point l_pt = pt * robot_tf;

      geometry_msgs::Point viz_pt;
      std_msgs::ColorRGBA color;
      color.r = i / 10.;
      color.g = i / 10.;
      color.b = i / 10.;
      color.a = 1.0;
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;

      viz_pts.points.push_back(viz_pt);
      viz_pts.colors.push_back(color);
    }
    if (plane.extremas_.size() > 1)
    {
      Point p_pt = plane.extremas_[0] * robot_tf;
      for (size_t k = 1; k < plane.extremas_.size(); k++)
      {
        Point l_pt = plane.extremas_[k] * robot_tf;

        std_msgs::ColorRGBA color;
        color.r = static_cast<int>(i + 1) % 2;
        color.g = static_cast<int>(i + 1) % 3;
        color.b = static_cast<int>(i + 1) % 4;
        color.a = 1.0;

        geometry_msgs::Point viz_pt;
        viz_pt.x = p_pt.x_;
        viz_pt.y = p_pt.y_;
        viz_pt.z = p_pt.z_;
        viz_line.points.push_back(viz_pt);
        viz_pt.x = l_pt.x_;
        viz_pt.y = l_pt.y_;
        viz_pt.z = l_pt.z_;
        viz_line.points.push_back(viz_pt);
        viz_line.color = color;
        viz_line.id = l;

        viz_pts.points.push_back(viz_pt);
        viz_pts.colors.push_back(color);
        marker_array.markers.push_back(viz_line);
        viz_line.points.clear();

        p_pt = l_pt;
        l += 1;
      }
    }

    if (plane.extremas_.size() > 1)
    {
      Point p_pt = plane.extremas_[0] * robot_tf;
      Point l_pt = plane.extremas_[plane.extremas_.size() - 1] * robot_tf;

      geometry_msgs::Point viz_pt;
      viz_pt.x = p_pt.x_;
      viz_pt.y = p_pt.y_;
      viz_pt.z = p_pt.z_;
      viz_line.points.push_back(viz_pt);
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;
      viz_line.points.push_back(viz_pt);
      viz_line.id = l++;
      marker_array.markers.push_back(viz_line);
      viz_line.points.clear();
    }

    Point centroid = plane.centroid_ * robot_tf;
    geometry_msgs::Point p1, p2;
    p1.x = centroid.x_;
    p1.y = centroid.y_;
    p1.z = centroid.z_;
    p2.x = p1.x + plane.a_;
    p2.y = p1.y + plane.b_;
    p2.z = p1.z + plane.c_;
    viz_normal.id = i;
    viz_normal.points.push_back(p1);
    viz_normal.points.push_back(p2);
    marker_array.markers.push_back(viz_normal);
    viz_normal.points.clear();

    i += 1;
    l += 1;
  }

  marker_array.markers.push_back(viz_pts);
  pub.publish(marker_array);
}

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<SemiPlane>& planes,
                                const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker viz_pts;
  visualization_msgs::Marker viz_line;
  visualization_msgs::Marker viz_normal;

  // Define markers layout

  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::Marker::POINTS;
  viz_pts.action = visualization_msgs::Marker::ADD;
  viz_pts.scale.x = 0.1;
  viz_pts.header.frame_id = params_.world_frame_id_;

  viz_line.ns = "/plane_extremas";
  viz_line.type = visualization_msgs::Marker::LINE_STRIP;
  viz_line.action = visualization_msgs::Marker::ADD;
  viz_line.color.r = 0.0;
  viz_line.color.g = 0.0;
  viz_line.color.b = 1.0;
  viz_line.color.a = 1.0;
  viz_line.scale.x = 0.1;
  viz_line.scale.y = 0.1;
  viz_line.header.frame_id = params_.world_frame_id_;

  viz_normal.ns = "/plane_normal";
  viz_normal.type = visualization_msgs::Marker::LINE_STRIP;
  viz_normal.action = visualization_msgs::Marker::ADD;
  viz_normal.color.a = 1;
  viz_normal.color.r = 1;
  viz_normal.color.b = 0;
  viz_normal.color.g = 0;
  viz_normal.scale.x = 0.1;
  viz_normal.header.frame_id = params_.world_frame_id_;

  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ r_pose.x_, r_pose.y_, r_pose.z_ });

  float i = 0;
  float l = 0;
  for (const auto& plane : planes)
  {
    for (const auto& pt : plane.points_)
    {
      Point l_pt = pt * robot_tf;

      geometry_msgs::Point viz_pt;
      std_msgs::ColorRGBA color;
      color.r = i / 10.;
      color.g = i / 10.;
      color.b = i / 10.;
      color.a = 1.0;
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;

      viz_pts.points.push_back(viz_pt);
      viz_pts.colors.push_back(color);
    }

    if (plane.extremas_.size() > 1)
    {
      Point p_pt = plane.extremas_[0] * robot_tf;
      for (size_t k = 1; k < plane.extremas_.size(); k++)
      {
        Point l_pt = plane.extremas_[k] * robot_tf;

        std_msgs::ColorRGBA color;
        color.r = static_cast<int>(i + 1) % 2;
        color.g = static_cast<int>(i + 1) % 3;
        color.b = static_cast<int>(i + 1) % 4;
        color.a = 1.0;

        geometry_msgs::Point viz_pt;
        viz_pt.x = p_pt.x_;
        viz_pt.y = p_pt.y_;
        viz_pt.z = p_pt.z_;
        viz_line.points.push_back(viz_pt);
        viz_pt.x = l_pt.x_;
        viz_pt.y = l_pt.y_;
        viz_pt.z = l_pt.z_;
        viz_line.points.push_back(viz_pt);
        viz_line.color = color;
        viz_line.id = l;

        viz_pts.points.push_back(viz_pt);
        viz_pts.colors.push_back(color);
        marker_array.markers.push_back(viz_line);
        viz_line.points.clear();

        p_pt = l_pt;
        l += 1;
      }
    }

    if (plane.extremas_.size() > 1)
    {
      Point p_pt = plane.extremas_[0] * robot_tf;
      Point l_pt = plane.extremas_[plane.extremas_.size() - 1] * robot_tf;

      geometry_msgs::Point viz_pt;
      viz_pt.x = p_pt.x_;
      viz_pt.y = p_pt.y_;
      viz_pt.z = p_pt.z_;
      viz_line.points.push_back(viz_pt);
      viz_pt.x = l_pt.x_;
      viz_pt.y = l_pt.y_;
      viz_pt.z = l_pt.z_;
      viz_line.points.push_back(viz_pt);
      viz_line.id = l;
      marker_array.markers.push_back(viz_line);
      viz_line.points.clear();
    }

    Point centroid = plane.centroid_ * robot_tf;
    geometry_msgs::Point p1, p2;
    p1.x = centroid.x_;
    p1.y = centroid.y_;
    p1.z = centroid.z_;
    p2.x = p1.x + plane.a_;
    p2.y = p1.y + plane.b_;
    p2.z = p1.z + plane.c_;
    viz_normal.id = i;
    viz_normal.points.push_back(p1);
    viz_normal.points.push_back(p2);
    marker_array.markers.push_back(viz_normal);
    viz_normal.points.clear();

    i += 1;
    l += 1;
  }

  marker_array.markers.push_back(viz_pts);
  pub.publish(marker_array);
}

void VineSLAM_ros::publish3DMap(const std::vector<Corner>& corners,
                                const ros::Publisher& pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);

  std::array<float, 9> robot_R{};
  robot_pose_.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ robot_pose_.x_, robot_pose_.y_, robot_pose_.z_ });

  for (const auto& corner : corners)
  {
    Point l_pt = corner.pos_ * robot_tf;

    pcl::PointXYZI pcl_pt;
    pcl_pt.x = l_pt.x_;
    pcl_pt.y = l_pt.y_;
    pcl_pt.z = l_pt.z_;

    pcl_pt.intensity = static_cast<float>(corner.which_plane_) * 10.0f;
    cloud_out->points.push_back(pcl_pt);
  }

  cloud_out->header.frame_id = params_.world_frame_id_;
  sensor_msgs::PointCloud2 cloud_out2;
  pcl::toROSMsg(*cloud_out, cloud_out2);
  pub.publish(cloud_out2);
}

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<Corner>& corners,
                                const ros::Publisher& pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);

  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ r_pose.x_, r_pose.y_, r_pose.z_ });

  for (const auto& corner : corners)
  {
    Point l_pt = corner.pos_ * robot_tf;

    pcl::PointXYZI pcl_pt;
    pcl_pt.x = l_pt.x_;
    pcl_pt.y = l_pt.y_;
    pcl_pt.z = l_pt.z_;

    pcl_pt.intensity = static_cast<float>(corner.which_plane_) * 10.0f;
    cloud_out->points.push_back(pcl_pt);
  }

  cloud_out->header.frame_id = params_.world_frame_id_;
  sensor_msgs::PointCloud2 cloud_out2;
  pcl::toROSMsg(*cloud_out, cloud_out2);
  pub.publish(cloud_out2);
}

void VineSLAM_ros::publish3DMap(const std::vector<Planar>& planars,
                                const ros::Publisher& pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);

  std::array<float, 9> robot_R{};
  robot_pose_.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ robot_pose_.x_, robot_pose_.y_, robot_pose_.z_ });

  for (const auto& planar_feature : planars)
  {
    Point l_pt = planar_feature.pos_ * robot_tf;

    pcl::PointXYZI pcl_pt;
    pcl_pt.x = l_pt.x_;
    pcl_pt.y = l_pt.y_;
    pcl_pt.z = l_pt.z_;

    pcl_pt.intensity = static_cast<float>(planar_feature.which_plane_) * 10.0f;
    cloud_out->points.push_back(pcl_pt);
  }

  cloud_out->header.frame_id = params_.world_frame_id_;
  sensor_msgs::PointCloud2 cloud_out2;
  pcl::toROSMsg(*cloud_out, cloud_out2);
  pub.publish(cloud_out2);
}

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<Planar>& planars,
                                const ros::Publisher& pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);

  std::array<float, 9> robot_R{};
  r_pose.toRotMatrix(robot_R);
  Tf robot_tf(robot_R, std::array<float, 3>{ r_pose.x_, r_pose.y_, r_pose.z_ });

  for (const auto& planar_feature : planars)
  {
    Point l_pt = planar_feature.pos_ * robot_tf;

    pcl::PointXYZI pcl_pt;
    pcl_pt.x = l_pt.x_;
    pcl_pt.y = l_pt.y_;
    pcl_pt.z = l_pt.z_;

    pcl_pt.intensity = static_cast<float>(planar_feature.which_plane_) * 10.0f;
    cloud_out->points.push_back(pcl_pt);
  }

  cloud_out->header.frame_id = params_.world_frame_id_;
  sensor_msgs::PointCloud2 cloud_out2;
  pcl::toROSMsg(*cloud_out, cloud_out2);
  pub.publish(cloud_out2);
}

void VineSLAM_ros::make6DofMarker(visualization_msgs::InteractiveMarker& imarker, Pose pose,
                                  std::string marker_name)
{
  // Convert euler to quaternion
  tf2::Quaternion q;
  q.setRPY(pose.R_, pose.P_, pose.Y_);

  // Create and initialize the interactive marker
  imarker.header.frame_id = params_.world_frame_id_;
  imarker.pose.position.x = pose.x_;
  imarker.pose.position.y = pose.y_;
  imarker.pose.position.z = pose.z_;
  imarker.pose.orientation.x = q.getX();
  imarker.pose.orientation.y = q.getY();
  imarker.pose.orientation.z = q.getZ();
  imarker.pose.orientation.w = q.getW();
  imarker.scale = 2;
  imarker.name = marker_name;
  imarker.description = "6-DoF interactive marker";

  // Create and set controls
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;

  visualization_msgs::Marker marker_box;
  marker_box.type = visualization_msgs::Marker::SPHERE;
  marker_box.scale.x = imarker.scale * 0.3;
  marker_box.scale.y = imarker.scale * 0.3;
  marker_box.scale.z = imarker.scale * 0.3;
  marker_box.color.r = 1;
  marker_box.color.b = 0;
  marker_box.color.g = 0;
  marker_box.color.a = 0.9;

  control.markers.push_back(marker_box);
  imarker.controls.push_back(control);
  imarker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  control = visualization_msgs::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);

  control = visualization_msgs::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);

  control = visualization_msgs::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);

  control = visualization_msgs::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);

  control = visualization_msgs::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);

  control = visualization_msgs::InteractiveMarkerControl();
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  imarker.controls.push_back(control);
}

}  // namespace vineslam
