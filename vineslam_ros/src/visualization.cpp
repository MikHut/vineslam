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
  for (auto layer : *grid_map_)
  {
    // Compute map layer bounds
    float xmin = layer.second.origin_.x_;
    float xmax = xmin + layer.second.width_;
    float ymin = layer.second.origin_.y_;
    float ymax = xmin + layer.second.lenght_;
    float zmin = static_cast<float>(layer.first) * grid_map_->resolution_z_ + grid_map_->origin_.z_;
    for (float i = xmin; i < xmax - grid_map_->resolution_;)
    {
      for (float j = ymin; j < ymax - grid_map_->resolution_;)
      {
        int8_t number_objs = layer.second(i, j).landmarks_.size() + layer.second(i, j).surf_features_.size() +
                             layer.second(i, j).corner_features_.size();

        if (number_objs == 0)
        {
          j += grid_map_->resolution_;
          continue;
        }

        visualization_msgs::Marker cube_cell;
        cube_cell.header.frame_id = "map";
        cube_cell.header.stamp = ros::Time::now();
        cube_cell.id = idx;
        cube_cell.type = visualization_msgs::Marker::CUBE;
        cube_cell.action = visualization_msgs::Marker::ADD;
        cube_cell.color.a = 0.7;
        cube_cell.color.r = 0;
        cube_cell.color.b = 1 - (static_cast<float>(1) / static_cast<float>(number_objs));
        cube_cell.color.g = 0;
        cube_cell.pose.position.x = i;
        cube_cell.pose.position.y = j;
        cube_cell.pose.position.z = zmin;
        cube_cell.pose.orientation.x = 0;
        cube_cell.pose.orientation.y = 0;
        cube_cell.pose.orientation.z = 0;
        cube_cell.pose.orientation.w = 1;
        cube_cell.scale.x = grid_map_->resolution_;
        cube_cell.scale.y = grid_map_->resolution_;
        cube_cell.scale.z = grid_map_->resolution_z_;

        occ_map.markers.push_back(cube_cell);

        idx++;
        j += grid_map_->resolution_;
      }
      i += grid_map_->resolution_;
    }
  }

  // Publish the map
  grid_map_publisher_.publish(occ_map);
}

void VineSLAM_ros::publish2DMap(const Pose& pose, const std::vector<float>& bearings,
                                const std::vector<float>& depths) const
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray ellipse_array;
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
  marker.lifetime = ros::Duration();

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
  ellipse.lifetime = ros::Duration();

  // Publish markers
  int id = 1;
  for (auto& it : (*grid_map_)(0))
  {
    for (const auto& l_sfeature : it.landmarks_)
    {
      // Draw sfeature mean
      marker.id = id;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "map";
      marker.pose.position.x = l_sfeature.second.pos_.x_;
      marker.pose.position.y = l_sfeature.second.pos_.y_;
      marker.pose.position.z = l_sfeature.second.pos_.z_;

      marker_array.markers.push_back(marker);

      // Draw sfeature standard deviation
      tf2::Quaternion q;
      q.setRPY(0, 0, l_sfeature.second.gauss_.theta_);

      ellipse.id = id;
      ellipse.header.stamp = ros::Time::now();
      ellipse.header.frame_id = "map";
      ellipse.pose.position.x = l_sfeature.second.pos_.x_;
      ellipse.pose.position.y = l_sfeature.second.pos_.y_;
      ellipse.pose.position.z = l_sfeature.second.pos_.z_;
      ellipse.scale.x = 3 * l_sfeature.second.gauss_.stdev_.x_;
      ellipse.scale.y = 3 * l_sfeature.second.gauss_.stdev_.y_;
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
  q.setRPY(0, 0, pose.gaussian_dist_.theta_);

  ellipse.id = id;
  ellipse.header.stamp = ros::Time::now();
  ellipse.header.frame_id = "map";
  ellipse.pose.position.x = pose.x_;
  ellipse.pose.position.y = pose.y_;
  ellipse.pose.position.z = pose.z_;
  ellipse.scale.x = 3 * pose.gaussian_dist_.stdev_.x_;
  ellipse.scale.y = 3 * pose.gaussian_dist_.stdev_.y_;
  ellipse.pose.orientation.x = q.x();
  ellipse.pose.orientation.y = q.y();
  ellipse.pose.orientation.z = q.z();
  ellipse.pose.orientation.w = q.w();
  ellipse.color.r = 0.0f;
  ellipse.color.g = 0.0f;
  ellipse.color.b = 1.0f;
  ellipse.color.a = 1.0f;
  ellipse_array.markers.push_back(ellipse);

  map2D_publisher_.publish(marker_array);
  map2D_publisher_.publish(ellipse_array);
}

void VineSLAM_ros::publishElevationMap() const
{
  visualization_msgs::MarkerArray elevation_map_marker;
  visualization_msgs::Marker cube;

  float min_height = -0.3;
  float max_height = 0.3;

  // Define marker layout
  cube.ns = "/elevation_cube";
  cube.header.stamp = ros::Time::now();
  cube.header.frame_id = "map";
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;
  cube.color.a = 1.0;
  cube.lifetime = ros::Duration();

  // Compute map layer bounds
  float xmin = elevation_map_->origin_.x_;
  float xmax = xmin + elevation_map_->width_;
  float ymin = elevation_map_->origin_.y_;
  float ymax = xmin + elevation_map_->lenght_;
  std::cout << xmin << " , " << xmax << " | " << ymin << " , " << ymax << "\n";
  for (float i = xmin; i < xmax - grid_map_->resolution_;)
  {
    for (float j = ymin; j < ymax - grid_map_->resolution_;)
    {
      geometry_msgs::Point l_pt;
      l_pt.x = i;
      l_pt.y = j;
      l_pt.z = (*elevation_map_)(i, j);

      float r, g, b;
      float h = (1.0 - std::min(std::max((l_pt.z - min_height) / (max_height - min_height), 0.0), 1.0)) * 0.8;
      vineslam::ElevationMap::color(h, r, g, b);

      cube.color.r = r;
      cube.color.r = g;
      cube.color.r = b;
      cube.id = elevation_map_marker.markers.size();

      elevation_map_marker.markers.push_back(cube);

      j += grid_map_->resolution_;
    }
    i += grid_map_->resolution_;
  }

  elevation_map_publisher_.publish(elevation_map_marker);
}

void VineSLAM_ros::publish3DMap() const
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  std::vector<ImageFeature> surf_features = grid_map_->getImageFeatures();
  std::vector<Corner> corner_features = grid_map_->getCorners();
  std::vector<Planar> planar_features = grid_map_->getPlanars();

  for (const auto& feature : surf_features)
  {
    pcl::PointXYZRGB l_pt(feature.r_, feature.g_, feature.b_);
    l_pt.x = feature.pos_.x_;
    l_pt.y = feature.pos_.y_;
    l_pt.z = feature.pos_.z_;

    feature_cloud->points.push_back(l_pt);
  }

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

  feature_cloud->header.frame_id = "map";
  corner_cloud->header.frame_id = "map";
  planar_cloud->header.frame_id = "map";
  map3D_features_publisher_.publish(feature_cloud);
  map3D_corners_publisher_.publish(corner_cloud);
  map3D_planars_publisher_.publish(planar_cloud);
}

void VineSLAM_ros::publish3DMap(const std::vector<Plane>& planes, const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker viz_pts;

  // Define marker layout
  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::Marker::POINTS;
  viz_pts.action = visualization_msgs::Marker::ADD;
  viz_pts.scale.x = 0.1;
  viz_pts.scale.y = 0.1;
  viz_pts.lifetime = ros::Duration(0.2);
  viz_pts.header.frame_id = "map";

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

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<Plane>& planes, const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker viz_pts;

  // Define marker layout
  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::Marker::POINTS;
  viz_pts.action = visualization_msgs::Marker::ADD;
  viz_pts.scale.x = 0.1;
  viz_pts.scale.y = 0.1;
  viz_pts.lifetime = ros::Duration(0.2);
  viz_pts.header.frame_id = "map";

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

void VineSLAM_ros::publish3DMap(const std::vector<SemiPlane>& planes, const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker viz_pts;
  visualization_msgs::Marker viz_line, viz_line1;
  visualization_msgs::Marker viz_normal;

  // Define markers layout

  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::Marker::POINTS;
  viz_pts.action = visualization_msgs::Marker::ADD;
  viz_pts.lifetime = ros::Duration(0.2);
  viz_pts.scale.x = 0.1;
  viz_pts.header.frame_id = "map";

  viz_line.ns = "/plane_extremas";
  viz_line.type = visualization_msgs::Marker::LINE_STRIP;
  viz_line.action = visualization_msgs::Marker::ADD;
  viz_line.scale.x = 0.1;
  viz_line.color.r = 0.0;
  viz_line.color.g = 0.0;
  viz_line.color.b = 1.0;
  viz_line.color.a = 1.0;
  viz_line.lifetime = ros::Duration(0.2);
  viz_line.header.frame_id = "map";

  viz_normal.ns = "/plane_normal";
  viz_normal.type = visualization_msgs::Marker::LINE_STRIP;
  viz_normal.action = visualization_msgs::Marker::ADD;
  viz_normal.color.a = 1;
  viz_normal.color.r = 1;
  viz_normal.color.b = 0;
  viz_normal.color.g = 0;
  viz_normal.scale.x = 0.1;
  viz_normal.lifetime = ros::Duration(0.2);
  viz_normal.header.frame_id = "map";

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

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<SemiPlane>& planes, const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker viz_pts;
  visualization_msgs::Marker viz_line;
  visualization_msgs::Marker viz_normal;

  // Define markers layout

  viz_pts.ns = "/plane_pts";
  viz_pts.type = visualization_msgs::Marker::POINTS;
  viz_pts.action = visualization_msgs::Marker::ADD;
  viz_pts.lifetime = ros::Duration(0.2);
  viz_pts.scale.x = 0.1;
  viz_pts.header.frame_id = "map";

  viz_line.ns = "/plane_extremas";
  viz_line.type = visualization_msgs::Marker::LINE_STRIP;
  viz_line.action = visualization_msgs::Marker::ADD;
  viz_line.color.r = 0.0;
  viz_line.color.g = 0.0;
  viz_line.color.b = 1.0;
  viz_line.color.a = 1.0;
  viz_line.scale.x = 0.1;
  viz_line.scale.y = 0.1;
  viz_line.lifetime = ros::Duration(0.2);
  viz_line.header.frame_id = "map";

  viz_normal.ns = "/plane_normal";
  viz_normal.type = visualization_msgs::Marker::LINE_STRIP;
  viz_normal.action = visualization_msgs::Marker::ADD;
  viz_normal.color.a = 1;
  viz_normal.color.r = 1;
  viz_normal.color.b = 0;
  viz_normal.color.g = 0;
  viz_normal.scale.x = 0.1;
  viz_normal.lifetime = ros::Duration(0.2);
  viz_normal.header.frame_id = "map";

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

        std_msgs::ColorRGBA color;
        color.r = 1;
        color.g = 0;
        color.b = 0;
        color.a = 1.0;
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

void VineSLAM_ros::publish3DMap(const std::vector<Corner>& corners, const ros::Publisher& pub)
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

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<Corner>& corners, const ros::Publisher& pub)
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

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const std::vector<Planar>& planars, const ros::Publisher& pub)
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

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const Pose& r_pose, const std::vector<Planar>& planars, const ros::Publisher& pub)
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

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

}  // namespace vineslam