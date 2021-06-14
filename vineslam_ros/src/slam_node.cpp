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

  // Initialize variables
  estimate_heading_ = true;

  // Declare the Mappers and Localizer objects
  localizer_ = new Localizer(params_);
  land_mapper_ = new LandmarkMapper(params_);
  vis_mapper_ = new VisualMapper();
#if LIDAR_SENSOR == 0
  lid_mapper_ = new VelodyneMapper(params_);
#elif LIDAR_SENSOR == 1
  lid_mapper_ = new LivoxMapper(params_);
#endif
  timer_ = new Timer("VineSLAM subfunctions");

  // Image feature subscription
  occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/occupancy_map", 10,
      std::bind(&VineSLAM_ros::occupancyMapListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  feature_subscriber_ = this->create_subscription<vineslam_msgs::msg::FeatureArray>(
      "/features_topic", 10,
      std::bind(&VineSLAM_ros::imageFeatureListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // Semantic feature subscription
  landmark_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
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
  gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps_topic", 10,
      std::bind(&VineSLAM_ros::gpsListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // IMU subscriptions
  imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu_topic", 10,
      std::bind(&VineSLAM_ros::imuListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  imu_data_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_data_topic", 10,
      std::bind(&VineSLAM_ros::imuDataListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));

  // Publish maps and particle filter
  vineslam_report_publisher_ = this->create_publisher<vineslam_msgs::msg::Report>("/vineslam/report", 10);
  elevation_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/elevationMap", 10);
  semantic_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/map2D", 10);
  map3D_features_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/SURF", 10);
  map3D_corners_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/corners", 10);
  map3D_planars_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/planars", 10);
  map3D_planes_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/map3D/planes", 10);
  planes_local_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/map3D/planes_local", 10);
  semantic_local_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/map3D/semantic_local", 10);
  corners_local_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/corners_local", 10);
  planars_local_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/map3D/planars_local", 10);
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vineslam/pose", 10);
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/vineslam/path", 10);
  poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/vineslam/poses", 10);
  gps_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vineslam/gps_pose", 10);
  gps_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/vineslam/pose_fix", 10);
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
  geometry_msgs::msg::TransformStamped cam2base_msg, laser2base_msg;
  bool got_cam2base = false, got_laser2base = false;
  while (!got_cam2base && rclcpp::ok())
  {
    try
    {
      cam2base_msg = tf_buffer.lookupTransform(params_.camera_sensor_frame_, "base_link", rclcpp::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
      continue;
    }
    got_cam2base = true;
  }
  while (!got_laser2base && rclcpp::ok())
  {
    try
    {
      laser2base_msg = tf_buffer.lookupTransform(params_.lidar_sensor_frame_, "base_link", rclcpp::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
      continue;
    }
    got_laser2base = true;
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
  cam2base_tf_ = Pose(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw).toTf();

  tf2::Stamped<tf2::Transform> laser2base_stamped;
  tf2::fromMsg(laser2base_msg, laser2base_stamped);

  tf2::Transform laser2base = laser2base_stamped;  //.inverse();
  t = laser2base.getOrigin();
  laser2base.getBasis().getRPY(roll, pitch, yaw);

  lid_mapper_->setLaser2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);

  // Initialize tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Allocate map memory
  RCLCPP_INFO(this->get_logger(), "Allocating map memory!");
  grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0), 20, 5);
  elevation_map_ = new ElevationMap(params_, Pose(0, 0, 0, 0, 0, 0));
  RCLCPP_INFO(this->get_logger(), "Done!");

  // Call execution threads
  std::thread th1(&SLAMNode::loop, this);
  std::thread th2(&SLAMNode::publishDenseInfo, this, 1.);  // Publish dense info at 1Hz
  std::thread th3(&SLAMNode::broadcastTfs, this);
  th1.detach();
  th2.detach();
  th3.detach();
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
  param = prefix + ".camera_sensor_frame";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.camera_sensor_frame_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".lidar_sensor_frame";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.lidar_sensor_frame_))
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
  param = prefix + ".lightweight_version";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.lightweight_version_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".save_logs";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.save_logs_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".logs_folder";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.logs_folder_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.datum.latitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_lat_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.datum.longitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_long_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.datum.altitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_alt_))
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
  param = prefix + ".camera_info.horizontal_fov";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.h_fov_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".camera_info.vertical_fov";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.v_fov_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".camera_info.image_width";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.img_width_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".camera_info.image_height";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.img_height_))
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
                      (input_data_.received_landmarks_ || !params_.use_semantic_features_);  // &&
  //(input_data_.received_gnss_ || !params_.use_gps_);

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

void SLAMNode::init()
{
  // ---------------------------------------------------------
  // ----- Initialize the localizer and get first particles distribution
  // ---------------------------------------------------------
  localizer_->init(Pose(0, 0, 0, 0, 0, 0));
  localizer_->changeGPSFlag(false);  // We do not trust the GPS at the beginning since we still have to estimate
                                     // heading
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
#if LIDAR_SENSOR == 0
    lid_mapper_->localMap(input_data_.scan_pts_, l_corners, l_planars, l_planes, l_ground_plane);
#elif LIDAR_SENSOR == 1
    lid_mapper_->localMap(input_data_.scan_pts_, header_.stamp.sec, l_corners, l_planars, l_planes, l_ground_plane);
#endif
    //    l_planes = {};
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

  // Create logs file if desired
  if (params_.save_logs_)
  {
    n_saved_logs_ = 0;
    logs_file_.open(params_.logs_folder_ + "vineslam_logs.txt");
  }

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
#if LIDAR_SENSOR == 0
    lid_mapper_->localMap(input_data_.scan_pts_, l_corners, l_planars, l_planes, l_ground_plane);
#elif LIDAR_SENSOR == 1
    lid_mapper_->localMap(input_data_.scan_pts_, header_.stamp.sec, l_corners, l_planars, l_planes, l_ground_plane);
#endif
    //    l_planes = {};
    timer_->tock();
  }

  std::cout << "\n---\n" << l_corners.size() << ", " << l_planars.size() << "\n";

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
  //  odom_inc.x_ = -odom_inc.x_;
  input_data_.p_wheel_odom_pose_ = input_data_.wheel_odom_pose_;
  odom_inc.normalize();

  // Fuse odometry and gyroscope to get the innovation pose
  Pose innovation;
  if (params_.use_imu_)
  {
    computeInnovation(odom_inc, input_data_.imu_data_pose_, innovation);
  }
  else
  {
    innovation = odom_inc;
  }

  timer_->tick("localizer::process()");
  localizer_->process(innovation, obsv_, grid_map_);
  robot_pose_ = localizer_->getPose();
  timer_->tock();

  // ---------------------------------------------------------
  // ----- Reset IMU gyroscope angular integrators
  // ---------------------------------------------------------
  input_data_.imu_data_pose_ = Pose(0, 0, 0, 0, 0, 0);

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
  // ----- Conversion of pose into latitude and longitude
  // ---------------------------------------------------------
  double datum_utm_x, datum_utm_y, robot_utm_x, robot_utm_y;
  float robot_latitude, robot_longitude;
  std::string datum_utm_zone;
  Convertions::GNSS2UTM(params_.map_datum_lat_, params_.map_datum_long_, datum_utm_x, datum_utm_y, datum_utm_zone);
  robot_utm_x = datum_utm_x + (robot_pose_.x_ * std::cos(params_.map_datum_head_) -
                               robot_pose_.y_ * std::sin(params_.map_datum_head_));
  robot_utm_y = datum_utm_y - (robot_pose_.x_ * std::sin(params_.map_datum_head_) +
                               robot_pose_.y_ * std::cos(params_.map_datum_head_));
  Convertions::UTMtoGNSS(robot_utm_x, robot_utm_y, datum_utm_zone, robot_latitude, robot_longitude);

  // ---------------------------------------------------------
  // ----- ROS publishers
  // ---------------------------------------------------------
  tf2::Quaternion q;
  q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);
  q.normalize();

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

  // Publish robot pose in GNSS polar coordinates
  sensor_msgs::msg::NavSatFix pose_ll;
  pose_ll.header.stamp = header_.stamp;
  pose_ll.header.frame_id = "gps";
  pose_ll.latitude = robot_latitude;
  pose_ll.longitude = robot_longitude;
  gps_fix_publisher_->publish(pose_ll);

  // Non-dense publishers
  publish3DMap(l_corners, corners_local_publisher_);
  publish3DMap(l_planars, planars_local_publisher_);
  l_planes.push_back(l_ground_plane);
  publish3DMap(l_planes, planes_local_publisher_);
  publishRobotBox(robot_pose_);

  // Save logs if desired
  if (params_.save_logs_)
  {
    std::string pcl_file = "pcl_file_" + std::to_string(n_saved_logs_++) + ".pcd";

    // Write for the log file
    logs_file_ << robot_pose_.x_ << " " << robot_pose_.y_ << " " << robot_pose_.z_ << " " << robot_pose_.R_ << " "
               << robot_pose_.P_ << " " << robot_pose_.Y_ << "\n";

    // Save the pcl file
    pcl::PointCloud<pcl::PointXYZI> cloud;
    for (const auto& pt : input_data_.scan_pts_)
    {
      pcl::PointXYZI pcl_pt;
      pcl_pt.x = pt.x_;
      pcl_pt.y = pt.y_;
      pcl_pt.z = pt.z_;
      pcl_pt.intensity = pt.intensity_;
      cloud.push_back(pcl_pt);
    }
    pcl::io::savePCDFileASCII(params_.logs_folder_ + pcl_file, cloud);

    p_saved_pose_ = robot_pose_;
  }
}

void SLAMNode::broadcastTfs()
{
  uint32_t mil_secs = static_cast<uint32_t>((1 / 10) * 1e3);

  while (rclcpp::ok())
  {
    tf2::Quaternion q;

    // ---- base2map
    geometry_msgs::msg::TransformStamped base2map_msg;
    base2map_msg.header.stamp = header_.stamp;
    base2map_msg.header.frame_id = params_.world_frame_id_;
    base2map_msg.child_frame_id = "base_link";
    q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);
    q.normalize();
    Convertions::pose2TransformStamped(q, tf2::Vector3(robot_pose_.x_, robot_pose_.y_, robot_pose_.z_), base2map_msg);

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
      Convertions::pose2TransformStamped(q, tf2::Vector3(init_odom_pose_.x_, init_odom_pose_.y_, init_odom_pose_.z_),
                                         map2odom_msg);
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

        tf2::Stamped<tf2::Transform> odom2base_tf, base2map_tf, odom2map_tf;
        tf2::fromMsg(odom2base_msg, odom2base_tf);
        tf2::fromMsg(base2map_msg, base2map_tf);

        geometry_msgs::msg::TransformStamped odom2map_msg;

        tf2::Transform t = odom2base_tf * base2map_tf.inverse();
        odom2map_tf.setRotation(t.getRotation());
        odom2map_tf.setOrigin(t.getOrigin());
        Convertions::pose2TransformStamped(odom2map_tf.getRotation(), odom2map_tf.getOrigin(), odom2map_msg);

        odom2map_msg.header.stamp = header_.stamp;
        odom2map_msg.header.frame_id = "odom";
        odom2map_msg.child_frame_id = params_.world_frame_id_;

        tf_broadcaster_->sendTransform(odom2map_msg);
      }
      catch (tf2::TransformException& ex)
      {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      }
    }

    rclcpp::sleep_for(std::chrono::milliseconds(mil_secs));
  }
}

SLAMNode::~SLAMNode() = default;

}  // namespace vineslam
