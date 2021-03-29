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
  {
    Timer l_timer("VineSLAM main loop");
    l_timer.tick("vineslam_ros::process()");
    process();
    l_timer.tock();
    l_timer.getLog();
    l_timer.clearLog();
  }

  // Reset information flags
  input_data_.received_image_features_ = false;
  input_data_.received_scans_ = false;
  input_data_.received_landmarks_ = false;
  input_data_.received_odometry_ = false;
  input_data_.received_gnss_ = false;

  timer_->getLog();
  timer_->clearLog();
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
    timer_->tick("landmark_mapper::localMap()");
    land_mapper_->localMap(input_data_.land_bearings_, input_data_.land_depths_, l_landmarks);
    timer_->tock();
  }

  // - Compute 3D PCL corners and ground plane on robot's referential frame
  std::vector<Corner> l_corners;
  std::vector<Planar> l_planars;
  std::vector<SemiPlane> l_planes;
  SemiPlane l_ground_plane;
  if (params_.use_lidar_features_)
  {
    timer_->tick("lidar_mapper::localMap()");
    lid_mapper_->localMap(input_data_.scan_pts_, l_corners, l_planars, l_planes, l_ground_plane);
    timer_->tock();
  }

  // - Compute 3D image features on robot's referential frame
  std::vector<ImageFeature> l_surf_features;
  if (params_.use_image_features_)
  {
    timer_->tick("visual_mapper::localMap()");
    vis_mapper_->localMap(input_data_.image_features_, l_surf_features);
    timer_->tock();
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
  if (params_.use_gps_ && !init_gps_)
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

  timer_->tick("localizer::process()");
  localizer_->process(odom_inc, obsv_, grid_map_);
  robot_pose_ = localizer_->getPose();
  timer_->tock();

  // ---------------------------------------------------------
  // ----- Register multi-layer map (if performing SLAM)
  // ---------------------------------------------------------
  if (register_map_)
  {
    timer_->tick("landmark_mapper::process()");
    land_mapper_->process(robot_pose_, l_landmarks, input_data_.land_labels_, *grid_map_);
    timer_->tock();

    timer_->tick("visual_mapper::registerMaps()");
    vis_mapper_->registerMaps(robot_pose_, l_surf_features, *grid_map_);
    timer_->tock();

    timer_->tick("lidar_mapper::registerMaps()");
    lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, l_ground_plane, *grid_map_, *elevation_map_);
    timer_->tock();

    timer_->tick("grid_map::downsamplePlanars()");
    grid_map_->downsamplePlanars();
    timer_->tock();
  }

  // ---------------------------------------------------------
  // ----- ROS publishers and tf broadcasting
  // ---------------------------------------------------------

  // Publish tf::Transforms
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf2::Quaternion q;

  // ---- base2map
  geometry_msgs::msg::TransformStamped base2map_msg;
  base2map_msg.header.stamp = header_.stamp;
  base2map_msg.header.frame_id = params_.world_frame_id_;
  base2map_msg.child_frame_id = "base_link";
  q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);
  q.normalize();
  pose2TransformStamped(q, tf2::Vector3(robot_pose_.x_, robot_pose_.y_, robot_pose_.z_), base2map_msg);

  if (params_.robot_model_ == "agrob")  // in this configuration we broadcast a map->base_link tf
  {
    tf_broadcaster_->sendTransform(base2map_msg);

    // ---- odom2map
    geometry_msgs::msg::TransformStamped map2odom_msg;
    map2odom_msg.header.stamp = header_.stamp;
    map2odom_msg.header.frame_id = "odom";
    map2odom_msg.child_frame_id = params_.world_frame_id_;
    q.setRPY(init_odom_pose_.R_, init_odom_pose_.P_, init_odom_pose_.Y_);
    q.normalize();
    pose2TransformStamped(q, tf2::Vector3(init_odom_pose_.x_, init_odom_pose_.y_, init_odom_pose_.z_), map2odom_msg);
    tf_broadcaster_->sendTransform(map2odom_msg);
    // ----
  }
  else  // in this configuration we broadcast a odom->map tf
  {
    geometry_msgs::msg::TransformStamped odom2base_msg;
    tf2_ros::Buffer tf_buffer(this->get_clock());
    tf2_ros::TransformListener tfListener(tf_buffer);

    try
    {
      odom2base_msg = tf_buffer.lookupTransform("odom", "base_link", rclcpp::Time(0), rclcpp::Duration(300000000));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }

    tf2::Stamped<tf2::Transform> odom2base_tf, base2map_tf, odom2map_tf;
    tf2::fromMsg(odom2base_msg, odom2base_tf);
    tf2::fromMsg(base2map_msg, base2map_tf);

    geometry_msgs::msg::TransformStamped odom2map_msg;

    tf2::Transform t = odom2base_tf * base2map_tf.inverse();
    odom2map_tf.setRotation(t.getRotation());
    odom2map_tf.setOrigin(t.getOrigin());
    pose2TransformStamped(odom2map_tf.getRotation(), odom2map_tf.getOrigin(), odom2map_msg);

    odom2map_msg.header.stamp = header_.stamp;
    odom2map_msg.header.frame_id = "odom";
    odom2map_msg.child_frame_id = params_.world_frame_id_;

    tf_broadcaster_->sendTransform(odom2map_msg);
  }

  // Convert vineslam pose to ROS pose and publish it
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = rclcpp::Time();
  pose_stamped.header.frame_id = params_.world_frame_id_;
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
  ros_path.header.frame_id = params_.world_frame_id_;
  ros_path.poses = path_;
  path_publisher_->publish(ros_path);

  // Local map publishers
  publish3DMap(l_corners, corners_local_publisher_);
  publish3DMap(l_planars, planars_local_publisher_);
  l_planes.push_back(l_ground_plane);
  publish3DMap(l_planes, planes_local_publisher_);
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

void VineSLAM_ros::gpsListener(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  header_ = msg->header;

  // If it is the first iteration - initialize odometry origin
  if (init_gps_)
  {
    init_gps_pose_ = Pose(msg->pose.pose.position.x, msg->pose.pose.position.y, 0, 0, 0, 0);

    init_gps_ = false;
    return;
  }

  // Transform odometry msg to maps' referential frame
  tf2::Quaternion g2m_q;
  g2m_q.setRPY(init_gps_pose_.R_, init_gps_pose_.P_, init_gps_pose_.Y_);
  tf2::Transform gps2map(g2m_q, tf2::Vector3(init_gps_pose_.x_, init_gps_pose_.y_, init_gps_pose_.z_));

  tf2::Quaternion gps_q;
  gps_q.setX(0);
  gps_q.setY(0);
  gps_q.setZ(0);
  gps_q.setW(1);
  tf2::Transform gps_tf(gps_q, tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0));

  gps_tf = gps2map.inverseTimes(gps_tf);

  tf2::Vector3 trans = gps_tf.getOrigin();

//  input_data_.gnss_pose_ = Pose(trans.x(), trans.y(), 0, 0, 0, 0);

  input_data_.received_gnss_ = true;
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
  ros_poses.header.frame_id = params_.world_frame_id_;
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
  vineslam_report_publisher_->publish(report);
}

}  // namespace vineslam
