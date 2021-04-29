#include "../include/localization_node.hpp"
#include "../include/convertions.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS node
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<vineslam::LocalizationNode>());
  rclcpp::shutdown();

  return 0;
}

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Constructor and destructor
// --------------------------------------------------------------------------------

LocalizationNode::LocalizationNode() : VineSLAM_ros("LocalizationNode")
{
  // Load params
  loadParameters(params_);

  // Set initialization flags default values
  init_flag_ = true;
  init_gps_ = true;
  init_odom_ = true;

  // Initialize variables
  estimate_heading_ = false;

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
  gps_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/vineslam/pose_fix", 10);
  // Debug publishers
  grid_map_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/debug/grid_map_limits", 10);
  robot_box_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/vineslam/debug/robot_box", 10);

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

  // Save tfs
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

  // ---------------------------------------------------------
  // ----- Load map dimensions and initialize it
  // ---------------------------------------------------------
  ElevationMapParser elevation_map_parser(params_);
  MapParser map_parser(params_);
  if (!map_parser.parseHeader(&params_))
  {
    RCLCPP_ERROR(this->get_logger(), "Map input file not found.");
    return;
  }
  else
  {
    grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0), 1, 1);
    elevation_map_ = new ElevationMap(params_, Pose(0, 0, 0, 0, 0, 0));
  }

  // ---------------------------------------------------------
  // ----- Load the map from the xml input file
  // ---------------------------------------------------------
  if (!map_parser.parseFile(&(*grid_map_)))
  {
    RCLCPP_ERROR(this->get_logger(), "Map input file not found.");
    return;
  }
  if (!elevation_map_parser.parseFile(&(*elevation_map_)))
  {
    RCLCPP_ERROR(this->get_logger(), "Map input file not found.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Ready for execution...");

  // Call execution thread
  std::thread th1(&LocalizationNode::loop, this);
  std::thread th2(&LocalizationNode::publishDenseInfo, this, 1.0);  // Publish dense info at 1.0Hz
  std::thread th3(&LocalizationNode::broadcastTfs, this);
  th1.detach();
  th2.detach();
  th3.detach();
}

void LocalizationNode::loadParameters(Parameters& params)
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
  param = prefix + ".robot.latitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.robot_datum_lat_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".robot.longitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.robot_datum_long_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".robot.altitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.robot_datum_alt_))
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
  param = prefix + ".multilayer_mapping.grid_map.map_file_path";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_input_file_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.output_folder";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_output_folder_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.elevation_map_file_path";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.elevation_map_input_file_))
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

void LocalizationNode::loop()
{
  // Reset information flags
  input_data_.received_image_features_ = false;
  input_data_.received_scans_ = false;
  input_data_.received_landmarks_ = false;
  input_data_.received_odometry_ = false;
  input_data_.received_gnss_ = false;

  // ---------------------------------------------------------
  // ----- Declare the interactive marker that will initialize
  // ----- the robot pose on the map
  // ---------------------------------------------------------
  publish3DMap();
  initializeOnMap();

  while (rclcpp::ok())
  {
    loopOnce();
  }
}

void LocalizationNode::loopOnce()
{
  // Check if we have all the necessary data
  bool can_continue = (input_data_.received_image_features_ || (!params_.use_image_features_)) &&
                      input_data_.received_scans_ &&
                      (input_data_.received_landmarks_ || !params_.use_semantic_features_) &&
                      (input_data_.received_gnss_ || !params_.use_gps_);

  if (!can_continue)
    return;

  // VineSLAM main loop
  if (!init_flag_)
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

void LocalizationNode::init()
{
  // ---------------------------------------------------------
  // ----- Initialize the localizer and get first particles distribution
  // ---------------------------------------------------------
  localizer_->init(robot_pose_);
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

  RCLCPP_INFO(this->get_logger(), "Localization has started.");
}

void LocalizationNode::initializeOnMap()
{
  // -------------------------------------------------------------------------------
  // ------ Compute the difference between map's and robot's datums
  // -------------------------------------------------------------------------------
  geodetic_converter_ = new Geodetic(params_.map_datum_lat_, params_.map_datum_long_, params_.map_datum_alt_);

  double robot_e, robot_n, robot_u;
  geodetic_converter_->geodetic2enu(params_.robot_datum_lat_, params_.robot_datum_long_, params_.robot_datum_alt_,
                                    robot_e, robot_n, robot_u);

  // Rotate the obtained point considering the gnss heading
  Pose heading_pose(0, 0, 0, 0, 0, -params_.map_datum_head_);
  Tf heading_tf = heading_pose.toTf();
  Point corrected_point = Point(robot_n, -robot_e, robot_u) * heading_tf;

  robot_pose_ = Pose(corrected_point.x_, corrected_point.y_, corrected_point.z_, 0, 0, 0);

  // Set the map -> robot gnss transformation
  tf2::Quaternion q;
  q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);

  // Create the interactive marker menu entries
  im_menu_handler_ = interactive_markers::MenuHandler();
  im_menu_handler_.insert("Call matcher", std::bind(&LocalizationNode::iMenuCallback, this, std::placeholders::_1));
  im_menu_handler_.insert("Set pose", std::bind(&LocalizationNode::iMenuCallback, this, std::placeholders::_1));
  im_menu_handler_.insert("Reset pose", std::bind(&LocalizationNode::iMenuCallback, this, std::placeholders::_1));

  // Declare the interactive marker server
  im_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "/vineslam/initialization_marker", get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_topics_interface(), get_node_services_interface());

  // Create the 6-DoF interactive marker
  std::string marker_name = "initialization_markers";
  visualization_msgs::msg::InteractiveMarker imarker;
  make6DofMarker(imarker, robot_pose_, marker_name);

  // Insert the interactive marker and menu into the server and associate them with the corresponding callback functions
  im_server_->insert(imarker);
  im_server_->setCallback(imarker.name, std::bind(&LocalizationNode::iMarkerCallback, this, std::placeholders::_1));
  im_menu_handler_.apply(*im_server_, imarker.name);

  // 'Commit' changes and send to all clients
  im_server_->applyChanges();
}

void LocalizationNode::process()
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
  // ----- Reset IMU gyroscope angular integrators
  // ---------------------------------------------------------
  input_data_.imu_data_pose_ = Pose(0, 0, 0, 0, 0, 0);

  // ---------------------------------------------------------
  // ----- Conversion of pose into latitude and longitude
  // ---------------------------------------------------------
  double datum_utm_x, datum_utm_y, robot_utm_x, robot_utm_y;
  float robot_latitude, robot_longitude;
  std::string datum_utm_zone;
  GNSS2UTM(params_.map_datum_lat_, params_.map_datum_long_, datum_utm_x, datum_utm_y, datum_utm_zone);
  robot_utm_x = datum_utm_x + (robot_pose_.x_ * std::cos(params_.map_datum_head_) -
                               robot_pose_.y_ * std::sin(params_.map_datum_head_));
  robot_utm_y = datum_utm_y - (robot_pose_.x_ * std::sin(params_.map_datum_head_) +
                               robot_pose_.y_ * std::cos(params_.map_datum_head_));
  UTMtoGNSS(robot_utm_x, robot_utm_y, datum_utm_zone, robot_latitude, robot_longitude);

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

  // Publish robot pose in GNSS polar coordinates
  sensor_msgs::msg::NavSatFix pose_ll;
  pose_ll.header.stamp = header_.stamp;
  pose_ll.header.frame_id = "gps";
  pose_ll.latitude = robot_latitude;
  pose_ll.longitude = robot_longitude;
  gps_fix_publisher_->publish(pose_ll);

  publish3DMap(l_planars, planars_local_publisher_);
}

void LocalizationNode::broadcastTfs()
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
      catch (tf2::TransformException& ex)
      {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      }
    }

    rclcpp::sleep_for(std::chrono::milliseconds(mil_secs));
  }
}

void LocalizationNode::iMarkerCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  // Save the robot initial pose set by the user
  tf2::Quaternion q(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z,
                    feedback->pose.orientation.w);
  tf2Scalar R, P, Y;
  tf2::Matrix3x3(q).getRPY(R, P, Y);

  robot_pose_.x_ = feedback->pose.position.x;
  robot_pose_.y_ = feedback->pose.position.y;
  robot_pose_.z_ = feedback->pose.position.z;
  robot_pose_.R_ = static_cast<float>(R);
  robot_pose_.P_ = static_cast<float>(P);
  robot_pose_.Y_ = static_cast<float>(Y);
}

void LocalizationNode::iMenuCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  if (feedback->menu_entry_id == 1)
  {
    // Save the marker pose as initial guess for the matcher
    tf2::Quaternion q(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z,
                      feedback->pose.orientation.w);
    tf2Scalar R, P, Y;
    tf2::Matrix3x3(q).getRPY(R, P, Y);

    Pose initial_guess;
    initial_guess.x_ = feedback->pose.position.x;
    initial_guess.y_ = feedback->pose.position.y;
    initial_guess.z_ = feedback->pose.position.z;
    initial_guess.R_ = static_cast<float>(R);
    initial_guess.P_ = static_cast<float>(P);
    initial_guess.Y_ = static_cast<float>(Y);

    Tf initial_guess_tf = initial_guess.toTf();

    // Compute the planar features to use
    std::vector<Corner> l_corners;
    std::vector<Planar> l_planars;
    std::vector<SemiPlane> l_planes;
    SemiPlane l_ground_plane;
    if (params_.use_lidar_features_)
    {
      lid_mapper_->localMap(input_data_.scan_pts_, l_corners, l_planars, l_planes, l_ground_plane);
    }

    // Prepare and call the matcher
    ICP<Planar> initialization_matcher;
    initialization_matcher.setTolerance(1e-5);
    initialization_matcher.setMaxIterations(500);
    initialization_matcher.setRejectOutliersFlag(false);
    initialization_matcher.setInputTarget(grid_map_);
    initialization_matcher.setInputSource(l_planars);

    std::vector<Planar> aligned;
    float rms_error;
    if (initialization_matcher.align(initial_guess_tf, rms_error, aligned))
    {
      // Get homogeneous transformation result
      Tf result;
      initialization_matcher.getTransform(result);

      // Save the result into the robot pose
      robot_pose_ = Pose(result.R_array_, result.t_array_);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "The matcher failed, please try again with another initial guess.");
    }
  }
  else if (feedback->menu_entry_id == 2)
  {
    // Initialize the localization system
    RCLCPP_INFO(this->get_logger(), "Initializing system...");
    init();
    RCLCPP_INFO(this->get_logger(), "Initialization performed! Starting execution.");
    init_flag_ = false;
  }
  else if (feedback->menu_entry_id == 3)
  {
  }
}

LocalizationNode::~LocalizationNode() = default;

}  // namespace vineslam
