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
  estimate_heading_ = false;
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
  input_data_.received_images_ = false;
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
  bool can_continue =
      (input_data_.received_images_ || (!params_.use_image_features_ && !params_.use_semantic_features_)) &&
      input_data_.received_scans_ && (input_data_.received_landmarks_ || !params_.use_semantic_features_) &&
      (input_data_.received_gnss_ || !params_.use_gps_);

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
    process();

  // Reset information flags
  input_data_.received_images_ = false;
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
  grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0));

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
  std::vector<Plane> l_planes;
  Plane l_ground_plane;
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
    vis_mapper_->localMap(input_data_.rgb_image_, input_data_.depth_array_, l_surf_features);
  }

  if (register_map_)
  {
    // - Register 3D maps
    vis_mapper_->registerMaps(robot_pose_, l_surf_features, *grid_map_);
    lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, *grid_map_);
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
    land_mapper_->localMap(input_data_.land_bearings_, input_data_.land_depths_, l_landmarks);
  }

  // - Compute 3D PCL corners and ground plane on robot's referential frame
  std::vector<Corner> l_corners;
  std::vector<Planar> l_planars;
  std::vector<Plane> l_planes;
  Plane l_ground_plane;
  std::vector<SemiPlane> l_semi_planes;
  Tf plane_ref;
  if (params_.use_lidar_features_)
  {
    lid_mapper_->localMap(input_data_.scan_pts_, l_corners, l_planars, l_planes, l_ground_plane);

    for (const auto& plane : l_planes)
    {
      SemiPlane l_semi_plane;
      LidarMapper::convexHull(plane, l_semi_plane, plane_ref);
      l_semi_planes.push_back(l_semi_plane);
    }
  }

  // - Compute 3D image features on robot's referential frame
  std::vector<ImageFeature> l_surf_features;
  if (params_.use_image_features_)
  {
    vis_mapper_->localMap(input_data_.rgb_image_, input_data_.depth_array_, l_surf_features);
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
  localizer_->process(input_data_.wheel_odom_pose_, obsv_, previous_map_, grid_map_);
  robot_pose_ = localizer_->getPose();

  // ---------------------------------------------------------
  // ----- Register multi-layer map (if performing SLAM)
  // ---------------------------------------------------------
  if (register_map_)
  {
    land_mapper_->process(robot_pose_, l_landmarks, input_data_.land_labels_, *grid_map_);
    vis_mapper_->registerMaps(robot_pose_, l_surf_features, *grid_map_);
    lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, *grid_map_);
    grid_map_->downsamplePlanars();
  }

  // ---------------------------------------------------------
  // ----- ROS publishers and tf broadcasting
  // ---------------------------------------------------------

  // Convert robot pose to tf::Transform corresponding
  tf::Quaternion q;
  q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);
  q.normalize();
  tf::Transform base2map;
  base2map.setRotation(q);
  base2map.setOrigin(tf::Vector3(robot_pose_.x_, robot_pose_.y_, robot_pose_.z_));

  // Publish tf::Trasforms
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(base2map, ros::Time::now(), "map", "/vineslam/base_link"));
  tf::Transform cam2base(
      tf::Quaternion(params_.cam2base_[3], params_.cam2base_[4], params_.cam2base_[5], params_.cam2base_[6]),
      tf::Vector3(params_.cam2base_[0], params_.cam2base_[1], params_.cam2base_[2]));
  br.sendTransform(
      tf::StampedTransform(cam2base, ros::Time::now(), "/vineslam/base_link", "zed_camera_left_optical_frame"));
  tf::Transform vel2base(
      tf::Quaternion(params_.vel2base_[3], params_.vel2base_[4], params_.vel2base_[5], params_.vel2base_[6]),
      tf::Vector3(params_.vel2base_[0], params_.vel2base_[1], params_.vel2base_[2]));
  br.sendTransform(tf::StampedTransform(vel2base, ros::Time::now(), "/vineslam/base_link", "velodyne"));
  tf::Quaternion o2m_q;
  o2m_q.setRPY(init_odom_pose_.R_, init_odom_pose_.P_, init_odom_pose_.Y_);
  tf::Transform odom2map(o2m_q, tf::Vector3(init_odom_pose_.x_, init_odom_pose_.y_, init_odom_pose_.z_));
  br.sendTransform(tf::StampedTransform(odom2map, ros::Time::now(), "odom", "map"));

  tf::Quaternion p2m_q;
  Pose plane_pose(plane_ref.R_array_, plane_ref.t_array_);
  p2m_q.setRPY(plane_pose.R_, plane_pose.P_, plane_pose.Y_);
  tf::Transform plane2map(p2m_q, tf::Vector3(plane_pose.x_, plane_pose.y_, plane_pose.z_));
  br.sendTransform(tf::StampedTransform(plane2map, ros::Time::now(), "/vineslam/base_link", "plane"));

  // Convert vineslam pose to ROS pose and publish it
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = "map";
  pose_stamped.pose.position.x = robot_pose_.x_;
  pose_stamped.pose.position.y = robot_pose_.y_;
  pose_stamped.pose.position.z = robot_pose_.z_;
  pose_stamped.pose.orientation.x = q.x();
  pose_stamped.pose.orientation.y = q.y();
  pose_stamped.pose.orientation.z = q.z();
  pose_stamped.pose.orientation.w = q.w();
  pose_publisher_.publish(pose_stamped);

  // Push back the current pose to the path container and publish it
  path_.push_back(pose_stamped);
  nav_msgs::Path ros_path;
  ros_path.header.stamp = ros::Time::now();
  ros_path.header.frame_id = "map";
  ros_path.poses = path_;
  path_publisher_.publish(ros_path);

  // Publish particle poses (after and before resampling)
  // - Get the particles
  std::vector<Particle> b_particles, a_particles;
  (*localizer_).getParticlesBeforeResampling(b_particles);
  (*localizer_).getParticles(a_particles);
  // - Convert them to ROS pose array and fill the vineslam report msgs
  vineslam_msgs::report report;
  geometry_msgs::PoseArray ros_poses;
  ros_poses.header.stamp = ros::Time::now();
  ros_poses.header.frame_id = "map";
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

  // Publish the 2D map
  publish2DMap(robot_pose_, input_data_.land_bearings_, input_data_.land_depths_);
  // Publish 3D maps
  //  publish3DMap();
  publish3DMap(l_corners, corners_local_publisher_);
  publish3DMap(l_planars, planars_local_publisher_);
  std::vector<Plane> planes = { l_ground_plane };
  for (const auto& plane : l_planes)
    planes.push_back(plane);
  publish3DMap(l_semi_planes, planes_local_publisher_);

  // - Save local map for next iteration
  previous_map_->clear();
  for (const auto& planar : l_planars)
    previous_map_->insert(planar);
  for (const auto& corner : l_corners)
    previous_map_->insert(corner);
  previous_map_->downsamplePlanars();
}

void VineSLAM_ros::imageListener(const sensor_msgs::ImageConstPtr& rgb_image,
                                 const sensor_msgs::ImageConstPtr& depth_image)
{
  cv::Mat cv_image = cv_bridge::toCvShare(rgb_image, sensor_msgs::image_encodings::BGR8)->image;
  _imageListener(cv_image, depth_image);
}

void VineSLAM_ros::_imageListener(const cv::Mat& rgb_image, const sensor_msgs::ImageConstPtr& depth_image)
{
  input_data_.rgb_image_ = rgb_image;
  input_data_.depth_array_ = (float*)(&(depth_image)->data[0]);

  input_data_.received_images_ = true;
}

void VineSLAM_ros::landmarkListener(const vision_msgs::Detection2DArrayConstPtr& dets)
{
  // Declaration of the arrays that will constitute the SLAM observations
  std::vector<int> labels;
  std::vector<float> bearings;
  std::vector<float> depths;

  // -------------------------------------------------------------------------------
  // ---- Extract high-level semantic features
  // -------------------------------------------------------------------------------
  // Loop over all the bounding box detections
  if (dets != nullptr && input_data_.received_images_)
  {
    for (const auto& detection : (*dets).detections)
    {
      // Load a single bounding box detection
      vision_msgs::BoundingBox2D l_bbox = detection.bbox;

      // Calculate the bearing and depth of the detected object
      float depth;
      float bearing;

      // Compute bounding box limits
      auto xmin = static_cast<int>(l_bbox.center.x - l_bbox.size_x / 2);
      auto ymin = static_cast<int>(l_bbox.center.y - l_bbox.size_y / 2);
      auto xmax = static_cast<int>(l_bbox.center.x + l_bbox.size_x / 2);
      auto ymax = static_cast<int>(l_bbox.center.y + l_bbox.size_y / 2);

      // Set minimum and maximum depth values to consider
      float range_min = 0.01;
      float range_max = 10.0;

      std::map<float, float> dtheta;
      for (uint32_t i = xmin; i < xmax; i++)
      {
        for (uint32_t j = ymin; j < ymax; j++)
        {
          uint32_t idx = i + input_data_.rgb_image_.cols * j;

          // Fill the depth array with the values of interest
          if (std::isfinite(input_data_.depth_array_[idx]) && input_data_.depth_array_[idx] > range_min &&
              input_data_.depth_array_[idx] < range_max)
          {
            float x = input_data_.depth_array_[idx];
            float y = -(static_cast<float>(i) - params_.cx_) * (x / params_.fx_);
            auto l_depth = static_cast<float>(sqrt(pow(x, 2) + pow(y, 2)));
            dtheta[l_depth] = atan2(y, x);
          }
        }
      }

      // compute minimum of all observations
      size_t n_depths = dtheta.size();
      if (n_depths > 0)
      {
        depth = dtheta.begin()->first;
        bearing = dtheta.begin()->second;
      }
      else
      {
        depth = -1;
        bearing = -1;
      }

      // Check if the calculated depth is valid
      if (depth == -1)
        continue;

      // Insert the measures in the observations arrays
      labels.push_back(detection.results[0].id);
      depths.push_back(depth);
      bearings.push_back(bearing);
    }
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

void VineSLAM_ros::gpsListener(const sensor_msgs::NavSatFixConstPtr& msg)
{
  if (!params_.use_gps_)
    return;

  if (init_gps_)
  {
    // Set initial datum
    agrob_map_transform::SetDatum srv;
    srv.request.geo_pose.position.latitude = params_.latitude_;
    srv.request.geo_pose.position.longitude = params_.longitude_;
    srv.request.geo_pose.position.altitude = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(quat, srv.request.geo_pose.orientation);

    ROS_INFO("Setting GNSS datum...");
    set_datum_.call(srv);

    init_gps_ = false;
  }

  agrob_map_transform::GetPose srv;

  // GNSS - odom service call
  srv.request.geo_pose.latitude = msg->latitude;
  srv.request.geo_pose.longitude = msg->longitude;

  if (polar2pose_.call(srv))
  {
    Pose gps_odom;
    gps_odom.x_ = srv.response.local_pose.pose.pose.position.x;
    gps_odom.y_ = srv.response.local_pose.pose.pose.position.y;

    if (estimate_heading_)
      has_converged_ = getGNSSHeading(gps_odom, msg->header);

    // Compute the gnss to map transform
    tf::Quaternion heading_quat;
    heading_quat.setRPY(0., 0., heading_);
    heading_quat.normalize();
    tf::Transform ned2map(heading_quat, tf::Vector3(0., 0., 0.));

    // Publish gnss to map tf::Transform
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(ned2map, ros::Time::now(), "enu", "map"));

    // Publish gnss pose in the enu reference frame
    geometry_msgs::PoseStamped gnss_pose;
    gnss_pose.pose.position.x = gps_odom.x_;
    gnss_pose.pose.position.y = gps_odom.y_;
    gnss_pose.pose.position.z = gps_odom.z_;
    gnss_pose.pose.orientation.x = 0.;
    gnss_pose.pose.orientation.y = 0.;
    gnss_pose.pose.orientation.z = 0.;
    gnss_pose.pose.orientation.w = 1.;
    gnss_pose.header.stamp = ros::Time::now();
    gnss_pose.header.frame_id = "enu";

    gps_poses_.push_back(gnss_pose);
    nav_msgs::Path ros_path;
    ros_path.header.stamp = ros::Time::now();
    ros_path.header.frame_id = "map";
    ros_path.poses = gps_poses_;
    gps_publisher_.publish(ros_path);

    // Transform locally the gps pose from enu to map to use in localization
    tf::Matrix3x3 Rot = ned2map.getBasis().inverse();

    input_data_.gnss_pose_.x_ = static_cast<float>(Rot[0].getX()) * gps_odom.x_ +
                                static_cast<float>(Rot[0].getY()) * gps_odom.y_ +
                                static_cast<float>(Rot[0].getZ()) * gps_odom.z_;
    input_data_.gnss_pose_.y_ = static_cast<float>(Rot[1].getX()) * gps_odom.x_ +
                                static_cast<float>(Rot[1].getY()) * gps_odom.y_ +
                                static_cast<float>(Rot[1].getZ()) * gps_odom.z_;
    input_data_.gnss_pose_.z_ = 0.;
    input_data_.gnss_pose_.R_ = 0.;
    input_data_.gnss_pose_.P_ = 0.;
    input_data_.gnss_pose_.Y_ = 0.;
  }
  else
  {
    ROS_ERROR("Failed to call service Polar2Pose\n");
    return;
  }

  input_data_.received_gnss_ = true;
}

bool VineSLAM_ros::getGNSSHeading(const Pose& gps_odom, const std_msgs::Header& header)
{
  float weight_max = 0.;
  if (datum_autocorrection_stage_ == 0)
  {
    ROS_DEBUG("Initialization of AGROB DATUM");
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
          ROS_ERROR("Datum localization is bad. Error on heading location.");
          datum_autocorrection_stage_ = -1;
        }
      }
      else
      {
        ROS_ERROR("Error on heading location.");
        datum_autocorrection_stage_ = -1;
      }
    }
    else if (datum_autocorrection_stage_ == 2)
    {
      ROS_DEBUG("Initializing datum filter.");
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
        ROS_DEBUG("Solution = %d.", indexT);
      }
      else
        ROS_INFO("Did not find any solution for datum heading.");
    }
    else
      ROS_ERROR("Datum localization is bad. Error on heading location.");
  }

  return weight_max > 0.6;
}

}  // namespace vineslam
