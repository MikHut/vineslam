#include "../include/vineslam_ros.hpp"

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Callbacks and observation functions
// --------------------------------------------------------------------------------

bool VineSLAM_ros::startRegistration(vineslam_ros::start_map_registration::Request&,
                                     vineslam_ros::start_map_registration::Response&)
{
  ROS_INFO("Activating map registration ...\n");
  register_map_ = true;
  return true;
}

bool VineSLAM_ros::stopRegistration(vineslam_ros::stop_map_registration::Request&,
                                    vineslam_ros::stop_map_registration::Response&)
{
  ROS_INFO("Deactivating map registration ...\n");
  register_map_ = false;
  return true;
}

bool VineSLAM_ros::stopHeadingEstimation(vineslam_ros::stop_gps_heading_estimation::Request&,
                                         vineslam_ros::stop_gps_heading_estimation::Response&)
{
  ROS_INFO("Deactivating gps heading estimation ...\n");
  return true;
}

bool VineSLAM_ros::saveMap(vineslam_ros::save_map::Request&, vineslam_ros::save_map::Response&)
{
  ROS_INFO("Saving map to xml file.");

  // Save map data
  bool save_map = params_.save_map_;

  if (save_map)
  {
    MapWriter mw(params_);
    mw.writeToFile(grid_map_);
  }

  ROS_INFO("Map saved.");

  return true;
}

void VineSLAM_ros::loop()
{
  // Reset information flags
  input_data_.received_image_features_ = false;
  input_data_.received_scans_ = false;
  input_data_.received_landmarks_ = false;
  input_data_.received_odometry_ = false;
  input_data_.received_gnss_ = false;

  while (ros::ok())
  {
    loopOnce();
  }
}

void VineSLAM_ros::loopOnce()
{
  // Check if we have all the necessary data
  bool can_continue = (input_data_.received_image_features_ || (!params_.use_image_features_)) &&
                      input_data_.received_scans_ &&
                      (input_data_.received_landmarks_ || !params_.use_semantic_features_);

  if (!can_continue)
    return;

  // VineSLAM main loop
  if (init_flag_)
  {
    ROS_INFO("Initializing system...");
    init();
    ROS_INFO("Initialization performed!");
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

  ROS_INFO("Localization and Mapping has started.");
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
  timer_->tick("odom2map");
  Tf p_odom_tf = input_data_.p_wheel_odom_pose_.toTf();
  Tf c_odom_tf = input_data_.wheel_odom_pose_.toTf();
  Tf odom_inc_tf = p_odom_tf.inverse() * c_odom_tf;
  Pose odom_inc(odom_inc_tf.R_array_, odom_inc_tf.t_array_);
  odom_inc.normalize();
  timer_->tock();

  timer_->tick("localizer::process()");
  localizer_->process(odom_inc, obsv_, grid_map_);
  robot_pose_ = localizer_->getPose();
  timer_->tock();

  input_data_.p_wheel_odom_pose_ = input_data_.wheel_odom_pose_;

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
  timer_->tick("ros::publishers");

  static tf::TransformBroadcaster br;
  tf::Quaternion q;

  q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);
  tf::Transform base2map(q, tf::Vector3(robot_pose_.x_, robot_pose_.y_, robot_pose_.z_));
  br.sendTransform(tf::StampedTransform(base2map, ros::Time::now(), params_.world_frame_id_, "/base_link"));

  q.setRPY(init_odom_pose_.R_, init_odom_pose_.P_, init_odom_pose_.Y_);
  tf::Transform odom2map(q, tf::Vector3(init_odom_pose_.x_, init_odom_pose_.y_, init_odom_pose_.z_));
  br.sendTransform(tf::StampedTransform(odom2map, ros::Time::now(), "odom", params_.world_frame_id_));

  // Convert vineslam pose to ROS pose and publish it
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = params_.world_frame_id_;
  pose_stamped.pose.position.x = robot_pose_.x_;
  pose_stamped.pose.position.y = robot_pose_.y_;
  pose_stamped.pose.position.z = robot_pose_.z_;
  pose_stamped.pose.orientation.x = q.x();
  pose_stamped.pose.orientation.y = q.y();
  pose_stamped.pose.orientation.z = q.z();
  pose_stamped.pose.orientation.w = q.w();
  pose_publisher_.publish(pose_stamped);
  // Convert vineslam pose to ROS odometry pose and publish it
  nav_msgs::Odometry vineslam_odom;
  vineslam_odom.header.stamp = ros::Time::now();
  vineslam_odom.header.frame_id = params_.world_frame_id_;
  vineslam_odom.pose.pose.position.x = robot_pose_.x_;
  vineslam_odom.pose.pose.position.y = robot_pose_.y_;
  vineslam_odom.pose.pose.position.z = robot_pose_.z_;
  vineslam_odom.pose.pose.orientation.x = q.x();
  vineslam_odom.pose.pose.orientation.y = q.y();
  vineslam_odom.pose.pose.orientation.z = q.z();
  vineslam_odom.pose.pose.orientation.w = q.w();
  odom_publisher_.publish(vineslam_odom);

  // Push back the current pose to the path container and publish it
  path_.push_back(pose_stamped);
  nav_msgs::Path ros_path;
  ros_path.header.stamp = ros::Time::now();
  ros_path.header.frame_id = params_.world_frame_id_;
  ros_path.poses = path_;
  path_publisher_.publish(ros_path);

  // Publish local maps
  publish3DMap(l_corners, corners_local_publisher_);
  publish3DMap(l_planars, planars_local_publisher_);
  l_planes.push_back(l_ground_plane);
  publish3DMap(l_planes, planes_local_publisher_);

  timer_->tock();
}

void VineSLAM_ros::imageFeatureListener(const vineslam_msgs::FeatureArrayConstPtr& features)
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

void VineSLAM_ros::landmarkListener(const vision_msgs::Detection3DArrayConstPtr& dets)
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
    vision_msgs::BoundingBox3D l_bbox = detection.bbox;

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

void VineSLAM_ros::scanListener(const sensor_msgs::PointCloud2ConstPtr& msg)
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

void VineSLAM_ros::odomListener(const nav_msgs::OdometryConstPtr& msg)
{
  // If it is the first iteration - initialize odometry origin
  if (init_odom_)
  {
    // Convert odometry msg to pose msg
    tf::Pose pose_;
    geometry_msgs::Pose odom_pose = (*msg).pose.pose;
    tf::poseMsgToTF(odom_pose, pose_);

    // Check if yaw is NaN
    float yaw = static_cast<float>(tf::getYaw(pose_.getRotation()));
    if (!std::isfinite(yaw))
      yaw = 0;

    init_odom_pose_ = Pose(msg->pose.pose.position.x, msg->pose.pose.position.y, 0, 0, 0, yaw);
    input_data_.p_wheel_odom_pose_ = Pose(0, 0, 0, 0, 0, 0);

    init_odom_ = false;
    return;
  }

  // Transform odometry msg to maps' referential frame
  tf::Quaternion o2m_q;
  o2m_q.setRPY(init_odom_pose_.R_, init_odom_pose_.P_, init_odom_pose_.Y_);
  tf::Transform odom2map(o2m_q, tf::Vector3(init_odom_pose_.x_, init_odom_pose_.y_, init_odom_pose_.z_));

  tf::Quaternion odom_q;
  odom_q.setX(msg->pose.pose.orientation.x);
  odom_q.setY(msg->pose.pose.orientation.y);
  odom_q.setZ(msg->pose.pose.orientation.z);
  odom_q.setW(msg->pose.pose.orientation.w);
  tf::Transform odom_tf(odom_q,
                        tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

  odom_tf = odom2map.inverseTimes(odom_tf);

  tf::Vector3 trans = odom_tf.getOrigin();
  tf::Quaternion rot = odom_tf.getRotation();

  input_data_.wheel_odom_pose_ = Pose(trans.x(), trans.y(), 0, 0, 0, static_cast<float>(tf::getYaw(rot)));

  input_data_.received_odometry_ = true;
}

void VineSLAM_ros::gpsListener(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (!params_.use_gps_)
    return;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/enu", params_.world_frame_id_, now, ros::Duration(3.0));
    listener.lookupTransform("/enu", params_.world_frame_id_, now, transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  tf::Matrix3x3 Rot = transform.getBasis().inverse();

  tf::Quaternion q;
  q.setX(msg->pose.orientation.x);
  q.setY(msg->pose.orientation.y);
  q.setZ(msg->pose.orientation.z);
  q.setW(msg->pose.orientation.w);

  tfScalar r, p, y;
  tf::Matrix3x3 m(q);
  m.getRPY(r, p, y);

  Pose raw_gps_pose;
  raw_gps_pose.x_ = msg->pose.position.x;
  raw_gps_pose.y_ = msg->pose.position.y;
  raw_gps_pose.z_ = msg->pose.position.z;
  raw_gps_pose.R_ = r;
  raw_gps_pose.P_ = p;
  raw_gps_pose.Y_ = y;

  input_data_.gnss_pose_.x_ = static_cast<float>(Rot[0].getX()) * raw_gps_pose.x_ +
                              static_cast<float>(Rot[0].getY()) * raw_gps_pose.y_ +
                              static_cast<float>(Rot[0].getZ()) * raw_gps_pose.z_;
  input_data_.gnss_pose_.y_ = static_cast<float>(Rot[1].getX()) * raw_gps_pose.x_ +
                              static_cast<float>(Rot[1].getY()) * raw_gps_pose.y_ +
                              static_cast<float>(Rot[1].getZ()) * raw_gps_pose.z_;
  input_data_.gnss_pose_.z_ = 0.;
  input_data_.gnss_pose_.R_ = 0.;
  input_data_.gnss_pose_.P_ = 0.;
  input_data_.gnss_pose_.Y_ = 0.;

  input_data_.received_gnss_ = true;
  init_gps_ = false;
}

void VineSLAM_ros::publishReport() const
{
  // Publish particle poses (after and before resampling)
  // - Get the particles
  std::vector<Particle> b_particles, a_particles;
  (*localizer_).getParticlesBeforeResampling(b_particles);
  (*localizer_).getParticles(a_particles);
  // - Convert them to ROS pose array and fill the vineslam report msgs
  vineslam_msgs::report report;
  geometry_msgs::PoseArray ros_poses;
  ros_poses.header.stamp = ros::Time::now();
  ros_poses.header.frame_id = params_.world_frame_id_;
  report.header.stamp = ros_poses.header.stamp;
  report.header.frame_id = ros_poses.header.frame_id;
  for (const auto& particle : b_particles)
  {
    tf::Quaternion l_q;
    l_q.setRPY(particle.p_.R_, particle.p_.P_, particle.p_.Y_);
    l_q.normalize();

    geometry_msgs::Pose l_pose;
    l_pose.position.x = particle.p_.x_;
    l_pose.position.y = particle.p_.y_;
    l_pose.position.z = particle.p_.z_;
    l_pose.orientation.x = l_q.x();
    l_pose.orientation.y = l_q.y();
    l_pose.orientation.z = l_q.z();
    l_pose.orientation.w = l_q.w();

    vineslam_msgs::particle particle_info;
    particle_info.id = particle.id_;
    particle_info.pose = l_pose;
    particle_info.w = particle.w_;

    report.b_particles.push_back(particle_info);
  }
  for (const auto& particle : a_particles)
  {
    tf::Quaternion l_q;
    l_q.setRPY(particle.p_.R_, particle.p_.P_, particle.p_.Y_);
    l_q.normalize();

    geometry_msgs::Pose l_pose;
    l_pose.position.x = particle.p_.x_;
    l_pose.position.y = particle.p_.y_;
    l_pose.position.z = particle.p_.z_;
    l_pose.orientation.x = l_q.x();
    l_pose.orientation.y = l_q.y();
    l_pose.orientation.z = l_q.z();
    l_pose.orientation.w = l_q.w();

    ros_poses.poses.push_back(l_pose);

    vineslam_msgs::particle particle_info;
    particle_info.id = particle.id_;
    particle_info.pose = l_pose;
    particle_info.w = particle.w_;

    report.a_particles.push_back(particle_info);
  }
  poses_publisher_.publish(ros_poses);

  report.log.data = localizer_->logs_;
  report.use_semantic_features.data = params_.use_semantic_features_;
  report.use_lidar_features.data = params_.use_lidar_features_;
  report.use_image_features.data = params_.use_image_features_;
  report.use_gps.data = params_.use_gps_;
  vineslam_report_publisher_.publish(report);
}

}  // namespace vineslam
