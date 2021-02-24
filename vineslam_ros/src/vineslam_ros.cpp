#include "../include/vineslam_ros.hpp"

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Callbacks and observation functions
// --------------------------------------------------------------------------------

bool VineSLAM_ros::startRegistration(vineslam_ros::srv::StartMapRegistration::Request::SharedPtr,
                                     vineslam_ros::srv::StartMapRegistration::Response::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "Activating map registration ...\n");
  register_map_ = true;
  return true;
}

bool VineSLAM_ros::stopRegistration(vineslam_ros::srv::StopMapRegistration::Request::SharedPtr,
                                    vineslam_ros::srv::StopMapRegistration::Response::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating map registration ...\n");
  register_map_ = false;
  return true;
}

bool VineSLAM_ros::stopHeadingEstimation(vineslam_ros::srv::StopGpsHeadingEstimation::Request::SharedPtr,
                                         vineslam_ros::srv::StopGpsHeadingEstimation::Response::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating gps heading estimation ...\n");
  estimate_heading_ = false;
  return true;
}

bool VineSLAM_ros::saveMap(vineslam_ros::srv::SaveMap::Request::SharedPtr,
                           vineslam_ros::srv::SaveMap::Response::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "Saving map to xml file.");

  // Save map data
  bool save_map = params_.save_map_;

  if (save_map)
  {
    MapWriter mw(params_);
    mw.writeToFile(grid_map_);
  }

  RCLCPP_INFO(this->get_logger(), "Map saved.");

  return true;
}

void VineSLAM_ros::pose2TransformStamped(const tf2::Quaternion& q, const tf2::Vector3& t,
                                         geometry_msgs::msg::TransformStamped& tf)
{
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf.transform.translation.x = t.x();
  tf.transform.translation.y = t.y();
  tf.transform.translation.z = t.z();
}

void VineSLAM_ros::loop()
{
  // Reset information flags
  input_data_.received_image_features_ = false;
  input_data_.received_scans_ = false;
  input_data_.received_landmarks_ = false;
  input_data_.received_odometry_ = false;
  input_data_.received_gnss_ = false;

  while (rclcpp::ok())
  {
    loopOnce();
  }
}

void VineSLAM_ros::loopOnce()
{
  // Check if we have all the necessary data
  bool can_continue = (input_data_.received_image_features_ || (!params_.use_image_features_)) &&
                      input_data_.received_scans_ &&
                      (input_data_.received_landmarks_ || !params_.use_semantic_features_) &&
                      (input_data_.received_gnss_ || !params_.use_gps_);

  if (!can_continue)
    return;

  // VineSLAM main loop
  if (init_flag_)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing system...");
    init();
    RCLCPP_INFO(this->get_logger(), "Initialization performed! Starting execution.");
    init_flag_ = false;
  }
  else
    process();

  // Reset information flags
  input_data_.received_image_features_ = false;
  input_data_.received_scans_ = false;
  input_data_.received_landmarks_ = false;
  input_data_.received_odometry_ = false;
  input_data_.received_gnss_ = false;
}

void VineSLAM_ros::init()
{
  // ---------------------------------------------------------
  // ----- Initialize the localizer and get first particles distribution
  // ---------------------------------------------------------
  localizer_->init(Pose(0, 0, 0, 0, 0, 0));
  robot_pose_ = localizer_->getPose();

  // ---------------------------------------------------------
  // ----- Initialize the multi-layer maps
  // ---------------------------------------------------------

  // - 2D semantic feature map
  if (params_.use_semantic_features_)
  {
    land_mapper_->init(robot_pose_, input_data_.land_bearings_, input_data_.land_depths_, input_data_.land_labels_,
                       *grid_map_);
  }

  // - 3D PCL corner map estimation
  std::vector<Corner> l_corners;
  std::vector<Planar> l_planars;
  std::vector<SemiPlane> l_planes;
  SemiPlane l_ground_plane;
  if (params_.use_lidar_features_)
  {
    lid_mapper_->localMap(input_data_.scan_pts_, l_corners, l_planars, l_planes, l_ground_plane);

    // - Save local map for next iteration
    previous_map_->clear();
    for (const auto& planar : l_planars)
      previous_map_->insert(planar);
    for (const auto& corner : l_corners)
      previous_map_->insert(corner);
    previous_map_->downsamplePlanars();
  }

  // - 3D image feature map estimation
  std::vector<ImageFeature> l_surf_features;
  if (params_.use_image_features_)
  {
    vis_mapper_->localMap(input_data_.image_features_, l_surf_features);
  }

  if (register_map_)
  {
    // - Register 3D maps
    vis_mapper_->registerMaps(robot_pose_, l_surf_features, *grid_map_);
    lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, l_ground_plane, *grid_map_, *elevation_map_);
    grid_map_->downsamplePlanars();
  }

  RCLCPP_INFO(this->get_logger(), "Localization and Mapping has started.");
}

void VineSLAM_ros::process()
{
  // -------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------
  // ---- Localization and mapping procedures
  // -------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------

  // ---------------------------------------------------------
  // ----- Build local maps to use in the localization
  // ---------------------------------------------------------
  // - Compute 2D local map of semantic features on robot's referential frame
  std::vector<SemanticFeature> l_landmarks;
  if (params_.use_semantic_features_)
  {
    land_mapper_->localMap(input_data_.land_bearings_, input_data_.land_depths_, l_landmarks);
  }

  // - Compute 3D PCL corners and ground plane on robot's referential frame
  std::vector<Corner> l_corners;
  std::vector<Planar> l_planars;
  std::vector<SemiPlane> l_planes;
  SemiPlane l_ground_plane;
  if (params_.use_lidar_features_)
  {
    lid_mapper_->localMap(input_data_.scan_pts_, l_corners, l_planars, l_planes, l_ground_plane);
  }

  // - Compute 3D image features on robot's referential frame
  std::vector<ImageFeature> l_surf_features;
  if (params_.use_image_features_)
  {
    vis_mapper_->localMap(input_data_.image_features_, l_surf_features);
  }

  // ---------------------------------------------------------
  // ----- Build observation structure to use in the localization
  // ---------------------------------------------------------
  // * High level landmarks (if we're using them)
  // * Point cloud corners and planars
  // * SURF 3D image features
  // * GPS (if we're using it)
  obsv_.landmarks = l_landmarks;
  obsv_.corners = l_corners;
  obsv_.planars = l_planars;
  obsv_.ground_plane = l_ground_plane;
  obsv_.planes = l_planes;
  obsv_.surf_features = l_surf_features;
  if (has_converged_ && params_.use_gps_)
  {
    obsv_.gps_pose = input_data_.gnss_pose_;
  }
  else
    obsv_.gps_pose = Pose(0., 0., 0., 0., 0., 0.);

  // ---------------------------------------------------------
  // ----- Localization procedure
  // ---------------------------------------------------------
  Tf p_odom_tf = input_data_.p_wheel_odom_pose_.toTf();
  Tf c_odom_tf = input_data_.wheel_odom_pose_.toTf();
  Tf odom_inc_tf = p_odom_tf.inverse() * c_odom_tf;
  Pose odom_inc(odom_inc_tf.R_array_, odom_inc_tf.t_array_);
  input_data_.p_wheel_odom_pose_ = input_data_.wheel_odom_pose_;
  odom_inc.normalize();
  localizer_->process(odom_inc, obsv_, previous_map_, grid_map_);
  robot_pose_ = localizer_->getPose();

  // ---------------------------------------------------------
  // ----- Register multi-layer map (if performing SLAM)
  // ---------------------------------------------------------
  if (register_map_)
  {
    land_mapper_->process(robot_pose_, l_landmarks, input_data_.land_labels_, *grid_map_);
    vis_mapper_->registerMaps(robot_pose_, l_surf_features, *grid_map_);
    lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, l_ground_plane, *grid_map_, *elevation_map_);
    grid_map_->downsamplePlanars();
  }

  // ---------------------------------------------------------
  // ----- ROS publishers and tf broadcasting
  // ---------------------------------------------------------

  // Publish tf::Trasforms
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf2::Quaternion q;

  // ---- base2map
  geometry_msgs::msg::TransformStamped base2map_tf;
  base2map_tf.header.stamp = rclcpp::Time();
  base2map_tf.header.frame_id = "/map";
  base2map_tf.child_frame_id = "/base_link";
  q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);
  q.normalize();
  pose2TransformStamped(q, tf2::Vector3(robot_pose_.x_, robot_pose_.y_, robot_pose_.z_), base2map_tf);
  tf_broadcaster_->sendTransform(base2map_tf);

  // ---- odom2map
  geometry_msgs::msg::TransformStamped map2odom_tf;
  map2odom_tf.header.stamp = rclcpp::Time();
  map2odom_tf.header.frame_id = "/odom";
  map2odom_tf.child_frame_id = "/map";
  q.setRPY(init_odom_pose_.R_, init_odom_pose_.P_, init_odom_pose_.Y_);
  q.normalize();
  pose2TransformStamped(q, tf2::Vector3(init_odom_pose_.x_, init_odom_pose_.y_, init_odom_pose_.z_), map2odom_tf);
  tf_broadcaster_->sendTransform(map2odom_tf);
  // ----

  // Convert vineslam pose to ROS pose and publish it
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = rclcpp::Time();
  pose_stamped.header.frame_id = "map";
  pose_stamped.pose.position.x = robot_pose_.x_;
  pose_stamped.pose.position.y = robot_pose_.y_;
  pose_stamped.pose.position.z = robot_pose_.z_;
  pose_stamped.pose.orientation.x = q.x();
  pose_stamped.pose.orientation.y = q.y();
  pose_stamped.pose.orientation.z = q.z();
  pose_stamped.pose.orientation.w = q.w();
  pose_publisher_->publish(pose_stamped);

  // Push back the current pose to the path container and publish it
  path_.push_back(pose_stamped);
  nav_msgs::msg::Path ros_path;
  ros_path.header.stamp = rclcpp::Time();
  ros_path.header.frame_id = "map";
  ros_path.poses = path_;
  path_publisher_->publish(ros_path);

  // Publishes the VineSLAM report
  publishReport();

  // Publish the 2D map
  publish2DMap(robot_pose_, input_data_.land_bearings_, input_data_.land_depths_);
  // Publish 3D maps
  publish3DMap();
  publishElevationMap();
  publish3DMap(l_corners, corners_local_publisher_);
  publish3DMap(l_planars, planars_local_publisher_);
  l_planes.push_back(l_ground_plane);
  publish3DMap(l_planes, planes_local_publisher_);

  // - Prepare next iteration
  previous_map_->clear();
  for (const auto& planar : l_planars)
    previous_map_->insert(planar);
  for (const auto& corner : l_corners)
    previous_map_->insert(corner);
  previous_map_->downsamplePlanars();
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

void VineSLAM_ros::landmarkListener(const vision_msgs::msg::Detection3DArray::SharedPtr dets)
{
  // Declaration of the arrays that will constitute the SLAM observations
  std::vector<int> labels;
  std::vector<float> bearings;
  std::vector<float> depths;

  // -------------------------------------------------------------------------------
  // ---- Compute range-bearing representation of semantic features
  // -------------------------------------------------------------------------------
  for (const auto& detection : (*dets).detections)
  {
    vision_msgs::msg::BoundingBox3D l_bbox = detection.bbox;

    float x = l_bbox.center.position.z;
    float y = -(static_cast<float>(l_bbox.center.position.x) - params_.cx_) * (x / params_.fx_);

    auto depth = static_cast<float>(sqrt(pow(x, 2) + pow(y, 2)));
    float theta = atan2(y, x);

    // Insert the measures in the observations arrays
    //    labels.push_back(detection.results[0].id);
    labels.push_back(0);
    depths.push_back(depth);
    bearings.push_back(theta);
  }

  input_data_.land_labels_ = labels;
  input_data_.land_bearings_ = bearings;
  input_data_.land_depths_ = depths;

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
    Point l_pt(pt.x, pt.y, pt.z);
    input_data_.scan_pts_.push_back(l_pt);
  }

  input_data_.received_scans_ = true;
}

void VineSLAM_ros::odomListener(const nav_msgs::msg::Odometry::SharedPtr msg)
{
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
    float yaw = static_cast<float>(tf2::getYaw(q));
    if (!std::isfinite(yaw))
      yaw = 0;

    init_odom_pose_ = Pose(msg->pose.pose.position.x, msg->pose.pose.position.y, 0, 0, 0, yaw);
    input_data_.p_wheel_odom_pose_ = init_odom_pose_;

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
}

bool VineSLAM_ros::getGNSSHeading(const Pose& gps_odom, const std_msgs::msg::Header header)
{
  float weight_max = 0.;
  if (datum_autocorrection_stage_ == 0)
  {
    RCLCPP_DEBUG(this->get_logger(), "Initialization of AGROB DATUM");
    datum_autocorrection_stage_++;
  }
  else
  {
    float x, y;
    x = robot_pose_.x_;
    y = robot_pose_.y_;

    float distance = std::sqrt((gps_odom.x_ - x) * (gps_odom.x_ - x) + (gps_odom.y_ - y) * (gps_odom.y_ - y));
    float center_map = std::sqrt(gps_odom.x_ * gps_odom.x_ + gps_odom.y_ * gps_odom.y_);

    if (datum_autocorrection_stage_ == 1)
    {
      if (center_map < 2.0)
      {
        if (distance < 5.0)
        {
          datum_autocorrection_stage_ = 2;
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Datum localization is bad. Error on heading location.");
          datum_autocorrection_stage_ = -1;
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Error on heading location.");
        datum_autocorrection_stage_ = -1;
      }
    }
    else if (datum_autocorrection_stage_ == 2)
    {
      RCLCPP_DEBUG(this->get_logger(), "Initializing datum filter.");
      for (int i = 0; i < 360; i++)
      {
        datum_orientation_[i][0] = static_cast<float>(i);
        datum_orientation_[i][1] = 1.0;
      }
      datum_autocorrection_stage_ = 3;
    }
    else if (datum_autocorrection_stage_ == 3)
    {
      global_counter_++;

      float dist_temp_max = 0.0;
      for (auto& i : datum_orientation_)
      {
        float xtemp, ytemp, dist_temp;
        xtemp = std::cos(i[0] * DEGREE_TO_RAD) * x - std::sin(i[0] * DEGREE_TO_RAD) * y;
        ytemp = std::sin(i[0] * DEGREE_TO_RAD) * x + std::cos(i[0] * DEGREE_TO_RAD) * y;
        dist_temp =
            std::sqrt((gps_odom.x_ - xtemp) * (gps_odom.x_ - xtemp) + (gps_odom.y_ - ytemp) * (gps_odom.y_ - ytemp));
        i[2] = dist_temp;
        if (dist_temp_max < dist_temp)
          dist_temp_max = dist_temp;
      }

      int indexT = 0, index = 0;
      for (auto& i : datum_orientation_)
      {
        i[1] =
            (i[1] * static_cast<float>(global_counter_) + static_cast<float>(1. - i[2] / dist_temp_max) * center_map) /
            static_cast<float>(global_counter_ + center_map);

        if (weight_max < i[1])
        {
          weight_max = i[1];
          indexT = index;
        }

        index++;
      }

      if (weight_max > 0.)
      {
        heading_ = static_cast<float>(indexT) * DEGREE_TO_RAD;
        RCLCPP_DEBUG(this->get_logger(), "Solution = %d.", indexT);
      }
      else
        RCLCPP_INFO(this->get_logger(), "Did not find any solution for datum heading.");
    }
    else
      RCLCPP_ERROR(this->get_logger(), "Datum localization is bad. Error on heading location.");
  }

  return weight_max > 0.6;
}

void VineSLAM_ros::publishReport() const
{
  // Publish particle poses (after and before resampling)
  // - Get the particles
  std::vector<Particle> b_particles, a_particles;
  (*localizer_).getParticlesBeforeResampling(b_particles);
  (*localizer_).getParticles(a_particles);
  // - Convert them to ROS pose array and fill the vineslam report msgs
  vineslam_msgs::msg::Report report;
  geometry_msgs::msg::PoseArray ros_poses;
  ros_poses.header.stamp = rclcpp::Time();
  ros_poses.header.frame_id = "map";
  report.header.stamp = ros_poses.header.stamp;
  report.header.frame_id = ros_poses.header.frame_id;
  for (const auto& particle : b_particles)
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

    vineslam_msgs::msg::Particle particle_info;
    particle_info.id = particle.id_;
    particle_info.pose = l_pose;
    particle_info.w = particle.w_;

    report.b_particles.push_back(particle_info);
  }
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
  report.gps_heading = heading_;
  vineslam_report_publisher_->publish(report);
}

}  // namespace vineslam
