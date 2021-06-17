#include "../include/vineslam_ros.hpp"
#include "../include/convertions.hpp"

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Callbacks and observation functions
// --------------------------------------------------------------------------------

void VineSLAM_ros::occupancyMapListener(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Compute the difference between the satellite map origin and the slam's
  double n, e, d;
  Geodetic l_geodetic_converter(params_.sat_datum_lat_, params_.sat_datum_long_, params_.map_datum_alt_);
  l_geodetic_converter.geodetic2ned(params_.map_datum_lat_, params_.map_datum_long_, params_.map_datum_alt_, n, e, d);

  // Create the occupancy grid map that will be published
  Point map_origin(-e, -(msg->info.height * msg->info.resolution - std::fabs(n)), 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Set the 3D voxel map from the 2D occupancy grid map
  for (unsigned int x = 0; x < msg->info.width; x++)
  {
    for (unsigned int y = 0; y < msg->info.height; y++)
    {
      if (msg->data[x + msg->info.width * y] == 100)
      {
        // Convert the grid map indexes to real world coordinates
        float xx = (x + 0.5) * msg->info.resolution + map_origin.x_;
        float yy = (y + 0.5) * msg->info.resolution + map_origin.y_;

        // Transform the map occupied cells using the map datum heading
        float tf_xx =
            xx * std::cos(-params_.map_datum_head_ - M_PI / 2) - yy * std::sin(-params_.map_datum_head_ - M_PI / 2);
        float tf_yy =
            xx * std::sin(-params_.map_datum_head_ - M_PI / 2) + yy * std::cos(-params_.map_datum_head_ - M_PI / 2);

        if (grid_map_->isInside(tf_xx, tf_yy, 0.))
        {
          // Insert the point in the grid map to publish
          if ((x + msg->info.width * y) % 8 == 0)  // we do not show all the points for better performance
          {
            pcl::PointXYZ l_pt;
            l_pt.x = tf_xx;
            l_pt.y = tf_yy;
            l_pt.z = 0.;
            map_cloud->push_back(l_pt);
          }

          // Set the cell as occupied
          if ((*grid_map_)(tf_xx, tf_yy, 0).data == nullptr)
          {
            (*grid_map_)(tf_xx, tf_yy, 0).data = new CellData();
          }
          if ((*grid_map_)(tf_xx, tf_yy, 0).data->is_occupied_from_sat_ == nullptr)
          {
            (*grid_map_)(tf_xx, tf_yy, 0).data->is_occupied_from_sat_ = new bool();
          }

          *(*grid_map_)(tf_xx, tf_yy, 0).data->is_occupied_from_sat_ = true;
        }
      }
    }
  }

  // Grid map publish
  map_cloud->header.frame_id = params_.world_frame_id_;
  sensor_msgs::msg::PointCloud2 map_cloud2;
  pcl::toROSMsg(*map_cloud, map_cloud2);
  processed_occ_grid_publisher_->publish(map_cloud2);
}

void VineSLAM_ros::imageFeatureListener(const vineslam_msgs::msg::FeatureArray::SharedPtr features)
{
  std::vector<vineslam::ImageFeature> img_features;
  for (const auto& in_feature : features->features)
  {
    vineslam::ImageFeature l_ft;
    l_ft.u_ = in_feature.u;
    l_ft.v_ = in_feature.v;
    l_ft.r_ = in_feature.r;
    l_ft.g_ = in_feature.g;
    l_ft.b_ = in_feature.b;
    l_ft.pos_ = Point(in_feature.x, in_feature.y, in_feature.z);
    l_ft.laplacian_ = in_feature.laplacian;
    l_ft.signature_ = in_feature.signature;
    img_features.push_back(l_ft);
  }

  input_data_.image_features_ = img_features;
  input_data_.received_image_features_ = true;
}

void VineSLAM_ros::landmarkListener(const vision_msgs::msg::Detection2DArray::SharedPtr dets)
{
  // Declaration of the arrays that will constitute the SLAM observations
  std::vector<int> labels;
  std::vector<float> bearings;
  std::vector<float> pitches;

  // Compute range-bearing representation of semantic features
  for (const auto& detection : dets->detections)
  {
    vision_msgs::msg::BoundingBox2D l_bbox = detection.bbox;

    // Compute the pitch and yaw of each landmark
    int col = l_bbox.center.x;
    int row = l_bbox.center.y;
    float l_bearing = (-params_.h_fov_ / params_.img_width_) * (params_.img_width_ / 2 - col);
    float l_pitch = 0;
    if (std::stoi(detection.results[0].id) != 1)  // not a trunk
    {
      l_pitch = (-params_.v_fov_ / params_.img_height_) * (params_.img_height_ / 2 - row);
    }

    // Transform the landmark orientations into the robot referential frame
    Tf l_measurement_tf = Pose(0.0, 0.0, 0.0, 0.0, l_pitch, l_bearing).toTf();
    Tf l_measurement_base = cam2base_tf_.inverse() * l_measurement_tf;
    Pose l_measurement(l_measurement_base.R_array_, l_measurement_base.t_array_);

    // Save the observation
    std::cout << std::stoi(detection.results[0].id) <<  "----- \n";
    labels.push_back(std::stoi(detection.results[0].id));
    bearings.push_back(l_measurement.Y_);
    pitches.push_back(l_measurement.P_);
  }

  input_data_.land_labels_ = labels;
  input_data_.land_pitches_ = pitches;
  input_data_.land_bearings_ = bearings;

  input_data_.received_landmarks_ = true;
}

void VineSLAM_ros::scanListener(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_pcl(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *velodyne_pcl);
  // Remove Nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*velodyne_pcl, *velodyne_pcl, indices);

  input_data_.scan_pts_.clear();
  for (const auto& pt : *velodyne_pcl)
  {
    Point l_pt(pt.x, pt.y, pt.z, pt.intensity);
    input_data_.scan_pts_.push_back(l_pt);
  }

  input_data_.received_scans_ = true;
}

void VineSLAM_ros::odomListener(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  header_ = msg->header;

  // If it is the first iteration - initialize odometry origin
  if (init_odom_)
  {
    // Convert odometry msg to pose msg
    tf2::Quaternion q;
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    q.setW(msg->pose.pose.orientation.w);
    q.normalize();

    // Check if yaw is NaN
    auto yaw = static_cast<float>(tf2::getYaw(q));
    if (!std::isfinite(yaw))
      yaw = 0;

    init_odom_pose_ = Pose(msg->pose.pose.position.x, msg->pose.pose.position.y, 0, 0, 0, yaw);
    input_data_.p_wheel_odom_pose_ = Pose(0, 0, 0, 0, 0, 0);

    init_odom_ = false;
    return;
  }

  // Transform odometry msg to maps' referential frame
  tf2::Quaternion o2m_q;
  o2m_q.setRPY(init_odom_pose_.R_, init_odom_pose_.P_, init_odom_pose_.Y_);
  tf2::Transform odom2map(o2m_q, tf2::Vector3(init_odom_pose_.x_, init_odom_pose_.y_, init_odom_pose_.z_));

  tf2::Quaternion odom_q;
  odom_q.setX(msg->pose.pose.orientation.x);
  odom_q.setY(msg->pose.pose.orientation.y);
  odom_q.setZ(msg->pose.pose.orientation.z);
  odom_q.setW(msg->pose.pose.orientation.w);
  tf2::Transform odom_tf(odom_q,
                         tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

  odom_tf = odom2map.inverseTimes(odom_tf);

  tf2::Vector3 trans = odom_tf.getOrigin();
  tf2::Quaternion rot = odom_tf.getRotation();

  input_data_.wheel_odom_pose_ = Pose(trans.x(), trans.y(), 0, 0, 0, static_cast<float>(tf2::getYaw(rot)));

  input_data_.received_odometry_ = true;
}

void VineSLAM_ros::gpsListener(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  header_ = msg->header;

  if (init_gps_)
  {
    geodetic_converter_ = new Geodetic(params_.map_datum_lat_, params_.map_datum_long_, params_.map_datum_alt_);
    init_gps_ = false;
  }
  else
  {
    double e, n, u;
    geodetic_converter_->geodetic2enu(msg->latitude, msg->longitude, msg->altitude, e, n, u);

    // Save pose
    input_data_.gnss_raw_pose_.x_ = n;
    input_data_.gnss_raw_pose_.y_ = -e;
    input_data_.gnss_raw_pose_.z_ = u;

    std::cout << "\n\n\n" << params_.map_datum_head_ << "\n\n\n";
    if (estimate_heading_)
    {
      getGNSSHeading();
    }

    // Convert the gnss with the correct heading
    Pose heading_pose(0, 0, 0, 0, 0, params_.map_datum_head_);
    Tf heading_tf = heading_pose.toTf();
    Point corrected_gnss_pose =
        Point(input_data_.gnss_raw_pose_.x_, input_data_.gnss_raw_pose_.y_, input_data_.gnss_raw_pose_.z_) *
        heading_tf.inverse();
    input_data_.gnss_pose_.x_ = corrected_gnss_pose.x_;
    input_data_.gnss_pose_.y_ = corrected_gnss_pose.y_;
    input_data_.gnss_pose_.z_ = corrected_gnss_pose.z_;

    // Set received flag to true
    input_data_.received_gnss_ = true;

    // Publish gps pose
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = header_.stamp;
    pose_stamped.header.frame_id = params_.world_frame_id_;
    pose_stamped.pose.position.x = input_data_.gnss_pose_.x_;
    pose_stamped.pose.position.y = input_data_.gnss_pose_.y_;
    pose_stamped.pose.position.z = input_data_.gnss_pose_.z_;
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1;
    gps_pose_publisher_->publish(pose_stamped);
  }
}

void VineSLAM_ros::imuListener(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  input_data_.imu_pose_.x_ = 0;
  input_data_.imu_pose_.y_ = 0;
  input_data_.imu_pose_.z_ = 0;
  input_data_.imu_pose_.R_ = msg->vector.x;
  input_data_.imu_pose_.P_ = msg->vector.y;
  input_data_.imu_pose_.Y_ = 0;
}

void VineSLAM_ros::imuDataListener(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  auto c_imu_observation_timestamp = std::chrono::high_resolution_clock::now();

  if (!init_flag_)
  {
    double d = std::chrono::duration_cast<std::chrono::milliseconds>(c_imu_observation_timestamp -
                                                                     p_imu_observation_timestamp_)
                   .count();

    input_data_.imu_data_pose_.x_ = 0;
    input_data_.imu_data_pose_.y_ = 0;
    input_data_.imu_data_pose_.z_ = 0;
    input_data_.imu_data_pose_.R_ += msg->angular_velocity.x * (d / 1000);
    input_data_.imu_data_pose_.P_ += msg->angular_velocity.y * (d / 1000);
    input_data_.imu_data_pose_.Y_ += msg->angular_velocity.z * (d / 1000);
  }

  p_imu_observation_timestamp_ = c_imu_observation_timestamp;
}

void VineSLAM_ros::publishReport() const
{
  // Publish particle poses (after and before resampling)
  // - Get the particles
  std::vector<Particle> a_particles;
  (*localizer_).getParticles(a_particles);
  // - Convert them to ROS pose array and fill the vineslam report msgs
  vineslam_msgs::msg::Report report;
  geometry_msgs::msg::PoseArray ros_poses;
  ros_poses.header.stamp = rclcpp::Time();
  ros_poses.header.frame_id = params_.world_frame_id_;
  report.header.stamp = ros_poses.header.stamp;
  report.header.frame_id = ros_poses.header.frame_id;
  for (const auto& particle : a_particles)
  {
    tf2::Quaternion l_q;
    l_q.setRPY(particle.p_.R_, particle.p_.P_, particle.p_.Y_);
    l_q.normalize();

    geometry_msgs::msg::Pose l_pose;
    l_pose.position.x = particle.p_.x_;
    l_pose.position.y = particle.p_.y_;
    l_pose.position.z = particle.p_.z_;
    l_pose.orientation.x = l_q.x();
    l_pose.orientation.y = l_q.y();
    l_pose.orientation.z = l_q.z();
    l_pose.orientation.w = l_q.w();

    ros_poses.poses.push_back(l_pose);

    vineslam_msgs::msg::Particle particle_info;
    particle_info.id = particle.id_;
    particle_info.pose = l_pose;
    particle_info.w = particle.w_;

    report.a_particles.push_back(particle_info);
  }
  poses_publisher_->publish(ros_poses);

  report.log.data = localizer_->logs_;
  report.use_semantic_features.data = params_.use_semantic_features_;
  report.use_lidar_features.data = params_.use_lidar_features_;
  report.use_image_features.data = params_.use_image_features_;
  report.use_gps.data = params_.use_gps_;
  vineslam_report_publisher_->publish(report);
}

void VineSLAM_ros::computeInnovation(const Pose& wheel_odom_inc, const Pose& imu_rot_inc, Pose& output_pose)
{
  double diff_yaw = std::fabs(wheel_odom_inc.Y_ - imu_rot_inc.Y_);
  double imu_weight = 0;
  if (diff_yaw < (0.5 * DEGREE_TO_RAD))
  {
    imu_weight = 0.5;
  }
  else if (diff_yaw > (0.5 * DEGREE_TO_RAD) && diff_yaw < (1.5 * DEGREE_TO_RAD))
  {
    imu_weight = 0.8;
  }
  else
  {
    imu_weight = 1.0;
  }

  double s_odom_Y = std::sin(wheel_odom_inc.Y_) * (1 - imu_weight);
  double c_odom_Y = std::cos(wheel_odom_inc.Y_) * (1 - imu_weight);
  double s_imu_Y = std::sin(imu_rot_inc.Y_) * imu_weight;
  double c_imu_Y = std::cos(imu_rot_inc.Y_) * imu_weight;
  double hat_yaw = std::atan2(s_odom_Y + s_imu_Y, c_odom_Y + c_imu_Y);

  output_pose = Pose(wheel_odom_inc.x_, wheel_odom_inc.y_, 0, imu_rot_inc.R_, imu_rot_inc.P_, hat_yaw);
}

void VineSLAM_ros::getGNSSHeading()
{
  float robot_distance_traveleld = robot_pose_.norm3D();

  if (robot_distance_traveleld > 0.1 && robot_distance_traveleld < 5.0)
  {
    float phi = 0.;
    float min_dist = std::numeric_limits<float>::max();
    while (phi < 360.)
    {
      float x = std::cos(phi * DEGREE_TO_RAD) * robot_pose_.x_ - std::sin(phi * DEGREE_TO_RAD) * robot_pose_.y_;
      float y = std::sin(phi * DEGREE_TO_RAD) * robot_pose_.x_ + std::cos(phi * DEGREE_TO_RAD) * robot_pose_.y_;

      Point source(x, y, 0.);
      Point target(input_data_.gnss_raw_pose_.x_, input_data_.gnss_raw_pose_.y_, 0.);
      float dist = source.distanceXY(target);

      if (dist < min_dist)
      {
        min_dist = dist;
        params_.map_datum_head_ = Const::normalizeAngle(phi * DEGREE_TO_RAD);
      }

      phi += 0.1;
    }
  }
  else if (robot_distance_traveleld >= 5.0)
  {
    localizer_->changeGPSFlag(true);  // Set the confidence of the use of gps in the particle filter now that we have
                                      // estimated heading.
    estimate_heading_ = false;
  }
}

// ------------------------------------------------------------------------------------
// ----- ROS services
// ------------------------------------------------------------------------------------

bool VineSLAM_ros::saveMap(vineslam_ros::srv::SaveMap::Request::SharedPtr,
                           vineslam_ros::srv::SaveMap::Response::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "Saving map to xml file.");

  // ----------------------------------------------------
  // ------ Export maps on xml file format
  // ----------------------------------------------------
  std::time_t timestamp = std::time(nullptr);

  MapWriter mw(params_, timestamp);
  mw.writeToFile(grid_map_, params_);

  ElevationMapWriter ew(params_, timestamp);
  ew.writeToFile(elevation_map_, params_);

  // ----------------------------------------------------
  // ------ Export geo-referenced map images and info
  // ----------------------------------------------------

  std::ofstream infofile;
  infofile.open(params_.map_output_folder_ + "info_" + std::to_string(timestamp) + ".json");

  // Compute polar coordinates of center and corners of the map image
  double datum_utm_x, datum_utm_y;
  std::string datum_utm_zone;
  Convertions::GNSS2UTM(params_.map_datum_lat_, params_.map_datum_long_, datum_utm_x, datum_utm_y, datum_utm_zone);

  Point referenced_map_center(datum_utm_x + (grid_map_->width_ / 2 + grid_map_->origin_.x_),
                              datum_utm_y + (grid_map_->lenght_ / 2 + grid_map_->origin_.y_), 0);

  // Convert the map into a square to ease the rotation process
  float width_inc = 0, lenght_inc = 0;
  Point left_upper_corner, left_bottom_corner, right_upper_corner, right_bottom_corner;
  if (grid_map_->width_ > grid_map_->lenght_)
  {
    lenght_inc = grid_map_->width_ - grid_map_->lenght_;

    left_upper_corner = referenced_map_center + Point(-grid_map_->width_ / 2, grid_map_->lenght_ / 2 + lenght_inc);
    left_bottom_corner = referenced_map_center + Point(-grid_map_->width_ / 2, -grid_map_->lenght_ / 2);
    right_upper_corner = referenced_map_center + Point(grid_map_->width_ / 2, grid_map_->lenght_ / 2 + lenght_inc);
    right_bottom_corner = referenced_map_center + Point(grid_map_->width_ / 2, -grid_map_->lenght_ / 2);
  }
  else if (grid_map_->width_ < grid_map_->height_)
  {
    width_inc = grid_map_->lenght_ - grid_map_->width_;

    left_upper_corner = referenced_map_center + Point(-grid_map_->width_ / 2 - width_inc, grid_map_->lenght_ / 2);
    left_bottom_corner = referenced_map_center + Point(-grid_map_->width_ / 2 - width_inc, -grid_map_->lenght_ / 2);
    right_upper_corner = referenced_map_center + Point(grid_map_->width_ / 2, grid_map_->lenght_ / 2);
    right_bottom_corner = referenced_map_center + Point(grid_map_->width_ / 2, -grid_map_->lenght_ / 2);
  }
  else
  {
    left_upper_corner = referenced_map_center + Point(-grid_map_->width_ / 2, grid_map_->lenght_ / 2);
    left_bottom_corner = referenced_map_center + Point(-grid_map_->width_ / 2, -grid_map_->lenght_ / 2);
    right_upper_corner = referenced_map_center + Point(grid_map_->width_ / 2, grid_map_->lenght_ / 2);
    right_bottom_corner = referenced_map_center + Point(grid_map_->width_ / 2, -grid_map_->lenght_ / 2);
  }

  // Compute GNSS location of the four corners
  Point left_upper_corner_ll, right_upper_corner_ll, left_bottom_corner_ll, right_bottom_corner_ll;

  Convertions::UTMtoGNSS(left_upper_corner.x_, left_upper_corner.y_, datum_utm_zone, left_upper_corner_ll.x_,
                         left_upper_corner_ll.y_);
  Convertions::UTMtoGNSS(right_upper_corner.x_, right_upper_corner.y_, datum_utm_zone, right_upper_corner_ll.x_,
                         right_upper_corner_ll.y_);
  Convertions::UTMtoGNSS(left_bottom_corner.x_, left_bottom_corner.y_, datum_utm_zone, left_bottom_corner_ll.x_,
                         left_bottom_corner_ll.y_);
  Convertions::UTMtoGNSS(right_bottom_corner.x_, right_bottom_corner.y_, datum_utm_zone, right_bottom_corner_ll.x_,
                         right_bottom_corner_ll.y_);

  infofile << "left_upper:\n";
  infofile << std::setprecision(8) << "   latitude: " << left_upper_corner_ll.x_ << "\n"
           << "   longitude: " << left_upper_corner_ll.y_ << "\n";
  infofile << "right_bottom:\n";
  infofile << std::setprecision(8) << "   latitude: " << right_bottom_corner_ll.x_ << "\n"
           << "   longitude: " << right_bottom_corner_ll.y_ << "\n";
  infofile.close();

  // Draw map images
  cv::Mat corners_image_map = cv::Mat(cv::Size((grid_map_->width_ + width_inc) / grid_map_->resolution_,
                                               (grid_map_->lenght_ + lenght_inc) / grid_map_->resolution_),
                                      CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Mat planars_image_map = cv::Mat(cv::Size((grid_map_->width_ + width_inc) / grid_map_->resolution_,
                                               (grid_map_->lenght_ + lenght_inc) / grid_map_->resolution_),
                                      CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Mat elevation_image_map = cv::Mat(cv::Size((elevation_map_->width_ + width_inc) / elevation_map_->resolution_,
                                                 (elevation_map_->lenght_ + lenght_inc) / elevation_map_->resolution_),
                                        CV_8UC3, cv::Scalar(255, 255, 255));

  // Corners image map
  std::vector<Corner> corners = grid_map_->getCorners();
  for (const auto& corner : corners)
  {
    cv::Point pt(static_cast<int>((corner.pos_.x_ - grid_map_->origin_.x_ + width_inc) / grid_map_->resolution_),
                 corners_image_map.rows -
                     static_cast<int>((corner.pos_.y_ - grid_map_->origin_.y_ + lenght_inc) / grid_map_->resolution_));

    corners_image_map.at<cv::Vec3b>(pt)[0] = 0;
    corners_image_map.at<cv::Vec3b>(pt)[1] = 255;
    corners_image_map.at<cv::Vec3b>(pt)[2] = 0;
  }
  // Planars image map
  std::vector<Planar> planars = grid_map_->getPlanars();
  for (const auto& planar : planars)
  {
    cv::Point pt(static_cast<int>((planar.pos_.x_ - grid_map_->origin_.x_ + width_inc) / grid_map_->resolution_),
                 planars_image_map.rows -
                     static_cast<int>((planar.pos_.y_ - grid_map_->origin_.y_ + lenght_inc) / grid_map_->resolution_));
    planars_image_map.at<cv::Vec3b>(pt)[0] = 0;
    planars_image_map.at<cv::Vec3b>(pt)[1] = 255;
    planars_image_map.at<cv::Vec3b>(pt)[2] = 0;
  }
  // Elevation image map
  float xmin = elevation_map_->origin_.x_;
  float xmax = xmin + elevation_map_->width_;
  float ymin = elevation_map_->origin_.y_;
  float ymax = ymin + elevation_map_->lenght_;
  float min_height = grid_map_->origin_.z_;
  float max_height = grid_map_->origin_.z_ + grid_map_->height_;
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
      else
      {
        cv::Point pt(static_cast<int>((i - elevation_map_->origin_.x_ + width_inc) / elevation_map_->resolution_),
                     elevation_image_map.rows -
                         static_cast<int>((j - elevation_map_->origin_.y_ + lenght_inc) / elevation_map_->resolution_));

        float h = (static_cast<float>(1) -
                   std::min(std::max((std::fabs(z) - min_height) / (max_height - min_height), static_cast<float>(0)),
                            static_cast<float>(1)));

        float r, g, b;
        vineslam::ElevationMap::color(h, r, g, b);
        int cv_r = static_cast<int>(r * 255), cv_g = static_cast<int>(g * 255), cv_b = static_cast<int>(b * 255);
        elevation_image_map.at<cv::Vec3b>(pt)[0] = cv_b;
        elevation_image_map.at<cv::Vec3b>(pt)[1] = cv_g;
        elevation_image_map.at<cv::Vec3b>(pt)[2] = cv_r;
      }

      j += elevation_map_->resolution_;
    }
    i += elevation_map_->resolution_;
  }

  // Rotate images by [heading] radians to align them with satellite images
  auto cv_rotate = [](cv::Mat input, float angle) {
    cv::Mat dst;
    cv::Point2f pt(input.cols / 2., input.rows / 2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(input, dst, r, cv::Size(input.cols, input.rows));
    return dst;
  };

  float heading = (params_.map_datum_head_ + M_PI / 2) * RAD_TO_DEGREE;
  cv::Mat ci = cv_rotate(corners_image_map, heading);
  cv::Mat pi = cv_rotate(planars_image_map, heading);
  cv::Mat ei = cv_rotate(elevation_image_map, heading);

  // Save images
  cv::imwrite(params_.map_output_folder_ + "corners_map_" + std::to_string(timestamp) + ".png", ci);
  cv::imwrite(params_.map_output_folder_ + "planars_map_" + std::to_string(timestamp) + ".png", pi);
  cv::imwrite(params_.map_output_folder_ + "elevation_map_" + std::to_string(timestamp) + ".png", ei);

  // ----------------------------------------------------
  // ------ Export 3D maps to pcd
  // ----------------------------------------------------
  pcl::PointCloud<pcl::PointXYZ> corners_cloud;
  for (const auto& c : corners)
  {
    pcl::PointXYZ pt;
    pt.x = c.pos_.x_;
    pt.y = c.pos_.y_;
    pt.z = c.pos_.z_;
    corners_cloud.push_back(pt);
  }
  pcl::io::savePCDFileASCII(params_.map_output_folder_ + "corners_map_" + std::to_string(timestamp) + ".pcd",
                            corners_cloud);

  pcl::PointCloud<pcl::PointXYZ> planars_cloud;
  for (const auto& p : planars)
  {
    pcl::PointXYZ pt;
    pt.x = p.pos_.x_;
    pt.y = p.pos_.y_;
    pt.z = p.pos_.z_;
    planars_cloud.push_back(pt);
  }
  pcl::io::savePCDFileASCII(params_.map_output_folder_ + "planars_map_" + std::to_string(timestamp) + ".pcd",
                            planars_cloud);

  pcl::PointCloud<pcl::PointXYZ> elevation_cloud;
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
      else
      {
        pcl::PointXYZ pt;
        pt.x = i + elevation_map_->resolution_ / 2;
        pt.y = j + elevation_map_->resolution_ / 2;
        pt.z = z;
        elevation_cloud.push_back(pt);
      }
      j += elevation_map_->resolution_;
    }
    i += elevation_map_->resolution_;
  }
  pcl::io::savePCDFileASCII(params_.map_output_folder_ + "elevation_map_" + std::to_string(timestamp) + ".pcd",
                            elevation_cloud);

  RCLCPP_INFO(this->get_logger(), "Maps saved.");

  return true;
}

}  // namespace vineslam