#include "../include/slam_node.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vineslam::SLAMNode>());
  rclcpp::shutdown();

  return 0;
}

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Constructor and destructor
// --------------------------------------------------------------------------------

SLAMNode::SLAMNode() : VineSLAM_ros("SLAMNode")
{
  // Load params
  loadParameters(params_);

  // Set initialization flags default values
  init_flag_ = true;
  init_gps_ = true;
  init_odom_ = true;

  // Set map to robot initial tf to 0 (it will always be zero since we are doing slam here)
  map2init = Pose(0, 0, 0, 0, 0, 0);

  // Declare the Mappers and Localizer objects
  localizer_ = new Localizer(params_);
  land_mapper_ = new LandmarkMapper(params_);
  vis_mapper_ = new VisualMapper();
  lid_mapper_ = new LidarMapper(params_);
  timer_ = new Timer("VineSLAM subfunctions");

  // Image feature subscription
  feature_subscriber_ = this->create_subscription<vineslam_msgs::msg::FeatureArray>(
      "/features_topic", 10,
      std::bind(&VineSLAM_ros::imageFeatureListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // Semantic feature subscription
  landmark_subscriber_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      "/detections_topic", 10,
      std::bind(&VineSLAM_ros::landmarkListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // Scan subscription
  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/scan_topic", 10,
      std::bind(&VineSLAM_ros::scanListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // Odometry subscription
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_topic", 10,
      std::bind(&VineSLAM_ros::odomListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // GPS subscription
  gps_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/gps_topic", 10,
      std::bind(&VineSLAM_ros::gpsListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // IMU subscription
  imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu_topic", 10,
      std::bind(&VineSLAM_ros::imuListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));

  // Publish maps and particle filter
  vineslam_report_publisher_ = this->create_publisher<vineslam_msgs::msg::Report>("/vineslam/report", 10);
  elevation_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/elevationMap", 10);
  map2D_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/map2D", 10);
  map3D_features_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/SURF", 10);
  map3D_corners_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/corners", 10);
  map3D_planars_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/planars", 10);
  map3D_planes_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/map3D/planes", 10);
  planes_local_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/map3D/planes_local", 10);
  corners_local_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/corners_local", 10);
  planars_local_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/planars_local", 10);
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vineslam/pose", 10);
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/vineslam/path", 10);
  poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/vineslam/poses", 10);
  gps_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vineslam/gps_pose", 10);
  // Debug publishers
  grid_map_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/debug/grid_map_limits", 10);
  robot_box_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/vineslam/debug/robot_box", 10);

  // ROS services
  save_map_srv_ = this->create_service<vineslam_ros::srv::SaveMap>(
      "/vineslam/save_map", std::bind(&VineSLAM_ros::saveMap, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1,
                                      std::placeholders::_2));

  // Static transforms
  RCLCPP_INFO(this->get_logger(), "Waiting for static transforms...");
  tf2_ros::Buffer tf_buffer(this->get_clock());
  tf2_ros::TransformListener tfListener(tf_buffer);
  geometry_msgs::msg::TransformStamped cam2base_msg, vel2base_msg;
  bool got_cam2base = false, got_vel2base = false;
  while (!got_cam2base && rclcpp::ok())
  {
    try
    {
      cam2base_msg = tf_buffer.lookupTransform("zed_camera_left_optical_frame", "base_link", rclcpp::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
      continue;
    }
    got_cam2base = true;
  }
  while (!got_vel2base && rclcpp::ok())
  {
    try
    {
      vel2base_msg = tf_buffer.lookupTransform("velodyne", "base_link", rclcpp::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
      continue;
    }
    got_vel2base = true;
  }
  RCLCPP_INFO(this->get_logger(), "Received!");

  // Save sensors to map transformation
  tf2::Stamped<tf2::Transform> cam2base_stamped;
  tf2::fromMsg(cam2base_msg, cam2base_stamped);

  tf2::Transform cam2base = cam2base_stamped;  //.inverse();
  tf2::Vector3 t = cam2base.getOrigin();
  tf2Scalar roll, pitch, yaw;
  cam2base.getBasis().getRPY(roll, pitch, yaw);
  vis_mapper_->setCam2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);
  land_mapper_->setCamPitch(pitch);

  tf2::Stamped<tf2::Transform> vel2base_stamped;
  tf2::fromMsg(vel2base_msg, vel2base_stamped);

  tf2::Transform vel2base = vel2base_stamped;  //.inverse();
  t = vel2base.getOrigin();
  vel2base.getBasis().getRPY(roll, pitch, yaw);
  lid_mapper_->setVel2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);

  // Initialize tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Allocate map memory
  RCLCPP_INFO(this->get_logger(), "Allocating map memory!");
  grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0), 20, 5);
  elevation_map_ = new ElevationMap(params_, Pose(0, 0, 0, 0, 0, 0));
  RCLCPP_INFO(this->get_logger(), "Done!");

  // Call execution thread
  std::thread th1(&SLAMNode::loop, this);
  std::thread th2(&SLAMNode::publishDenseInfo, this, 1.); // Publish dense info at 1Hz
  th1.detach();
  th2.detach();
}

void SLAMNode::loadParameters(Parameters& params)
{
  std::string prefix = this->get_name();
  std::string param;

  // Load params
  param = prefix + ".robot_model";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.robot_model_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".world_frame_id";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.world_frame_id_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".use_semantic_features";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_semantic_features_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".use_lidar_features";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_lidar_features_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".use_image_features";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_image_features_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".use_gps";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_gps_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".use_imu";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_imu_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".datum.latitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_lat_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".datum.longitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_long_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".datum.heading";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_head_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".camera_info.baseline";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.baseline_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".camera_info.fx";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.fx_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".robot_dimensions.x";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.robot_dim_x_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".robot_dimensions.y";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.robot_dim_y_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".robot_dimensions.z";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.robot_dim_z_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".camera_info.cx";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.cx_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.origin.x";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_origin_x_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.origin.y";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_origin_y_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.origin.z";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_origin_z_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.width";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_width_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.lenght";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_lenght_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.height";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_height_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.resolution";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_resolution_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.output_folder";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_output_folder_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".pf.n_particles";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.number_particles_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".pf.sigma_xx";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_xx_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".pf.sigma_yy";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_yy_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".pf.sigma_zz";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_zz_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".pf.sigma_RR";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_RR_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".pf.sigma_PP";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_PP_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".pf.sigma_YY";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_YY_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
}

void SLAMNode::loop()
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

void SLAMNode::loopOnce()
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
    //    l_timer.getLog();
    //    l_timer.clearLog();
  }

  // Reset information flags
  input_data_.received_image_features_ = false;
  input_data_.received_scans_ = false;
  input_data_.received_landmarks_ = false;
  input_data_.received_odometry_ = false;
  input_data_.received_gnss_ = false;

  //  timer_->getLog();
  //  timer_->clearLog();
}

void SLAMNode::init()
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
    l_planes = {};
  }

  // - 3D image feature map estimation
  std::vector<ImageFeature> l_surf_features;
  if (params_.use_image_features_)
  {
    vis_mapper_->localMap(input_data_.image_features_, l_surf_features);
  }

  // - Register 3D maps
  vis_mapper_->registerMaps(robot_pose_, l_surf_features, *grid_map_);
  lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, l_ground_plane, *grid_map_, *elevation_map_);
  grid_map_->downsamplePlanars();

  RCLCPP_INFO(this->get_logger(), "Localization and Mapping has started.");
}

void SLAMNode::process()
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
    l_planes = {};
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
  obsv_.landmarks_ = l_landmarks;
  obsv_.corners_ = l_corners;
  obsv_.planars_ = l_planars;
  obsv_.ground_plane_ = l_ground_plane;
  obsv_.planes_ = l_planes;
  obsv_.surf_features_ = l_surf_features;
  obsv_.gps_pose_ = input_data_.gnss_pose_;
  obsv_.imu_pose_ = input_data_.imu_pose_;

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

  // ---------------------------------------------------------
  // ----- ROS publishers and tf broadcasting
  // ---------------------------------------------------------

  // Publish tf::Transforms
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
  pose_stamped.header.stamp = header_.stamp;
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

  // Non-dense publishers
  publish3DMap(l_corners, corners_local_publisher_);
  publish3DMap(l_planars, planars_local_publisher_);
  l_planes.push_back(l_ground_plane);
  publish3DMap(l_planes, planes_local_publisher_);
  publishRobotBox(robot_pose_);
}

SLAMNode::~SLAMNode() = default;

}  // namespace vineslam
