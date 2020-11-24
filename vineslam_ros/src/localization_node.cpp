#include "../include/localization_node.hpp"

int main(int argc, char** argv)
{
  vineslam::LocalizationNode localization_node(argc, argv);
  return 0;
}

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Constructor and destructor
// --------------------------------------------------------------------------------

LocalizationNode::LocalizationNode(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "LocalizationNode");
  ros::NodeHandle nh;

  // Load params
  loadParameters(nh, "/localization_node", params_);

  // Set initialization flags default values
  init_flag_ = true;
  init_gps_ = true;
  init_odom_ = true;
  register_map_ = false;
  estimate_heading_ = true;

  // Declare the Mappers and Localizer objects
  localizer_ = new Localizer(params_);
  land_mapper_ = new LandmarkMapper(params_);
  vis_mapper_ = new VisualMapper(params_);
  lid_mapper_ = new LidarMapper(params_);

  // Initialize local grid map that will be used for relative motion calculation
  Parameters local_map_params;
  local_map_params.gridmap_origin_x_ = -30;
  local_map_params.gridmap_origin_y_ = -30;
  local_map_params.gridmap_origin_z_ = -0.5;
  local_map_params.gridmap_resolution_ = 0.20;
  local_map_params.gridmap_width_ = 60;
  local_map_params.gridmap_lenght_ = 60;
  local_map_params.gridmap_height_ = 2.5;
  previous_map_ = new OccupancyMap(local_map_params, Pose(0, 0, 0, 0, 0, 0));

  // Services
  polar2pose_ = nh.serviceClient<agrob_map_transform::GetPose>("polar_to_pose");
  set_datum_ = nh.serviceClient<agrob_map_transform::SetDatum>("datum");

  // Synchronize subscribers of both image topics
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh, params_.left_img_topic_, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, params_.depth_img_topic_, 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(left_image_sub, depth_image_sub, 10);
  sync.registerCallback(boost::bind(&LocalizationNode::imageListener, this, _1, _2));

  // Landmark subscription
  ros::Subscriber land_subscriber =
      nh.subscribe(params_.detections_topic_, 1, &VineSLAM_ros::landmarkListener, dynamic_cast<VineSLAM_ros*>(this));
  // Scan subscription
  ros::Subscriber scan_subscriber =
      nh.subscribe(params_.pcl_topic_, 1, &VineSLAM_ros::scanListener, dynamic_cast<VineSLAM_ros*>(this));
  // Odometry subscription
  ros::Subscriber odom_subscriber =
      nh.subscribe(params_.odom_topic_, 1, &VineSLAM_ros::odomListener, dynamic_cast<VineSLAM_ros*>(this));
  // GPS subscription
  ros::Subscriber gps_subscriber =
      nh.subscribe(params_.fix_topic_, 1, &VineSLAM_ros::gpsListener, dynamic_cast<VineSLAM_ros*>(this));

  // Publish maps and particle filter
  vineslam_report_publisher_ = nh.advertise<vineslam_msgs::report>("/vineslam/report", 1);
  grid_map_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/occupancyMap", 1);
  map2D_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map2D", 1);
  map3D_features_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/vineslam/map3D/SURF", 1);
  map3D_corners_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners", 1);
  map3D_planars_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars", 1);
  planes_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planes_local", 1);
  corners_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners_local", 1);
  planars_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars_local", 1);
  pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  gps_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/gps", 1);
  path_publisher_ = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher_ = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);

  // ROS services
  ros::ServiceServer stop_hed_srv = nh.advertiseService(
      "stop_gps_heading_estimation", &VineSLAM_ros::stopHeadingEstimation, dynamic_cast<VineSLAM_ros*>(this));

  // GNSS varibales
  if (params_.use_gps_)
  {
    datum_autocorrection_stage_ = 0;
    global_counter_ = 0;
  }

  // Get static sensor tfs
  tf::Transform cam2base;
  cam2base.setRotation(
      tf::Quaternion(params_.cam2base_[3], params_.cam2base_[4], params_.cam2base_[5], params_.cam2base_[6]));
  cam2base.setOrigin(tf::Vector3(params_.cam2base_[0], params_.cam2base_[1], params_.cam2base_[2]));
  cam2base = cam2base.inverse();
  tf::Vector3 t = cam2base.getOrigin();
  tfScalar roll, pitch, yaw;
  cam2base.getBasis().getRPY(roll, pitch, yaw);

  vis_mapper_->setCam2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);
  land_mapper_->setCamPitch(pitch);

  tf::Transform vel2base;
  vel2base.setRotation(
      tf::Quaternion(params_.vel2base_[3], params_.vel2base_[4], params_.vel2base_[5], params_.vel2base_[6]));
  vel2base.setOrigin(tf::Vector3(params_.vel2base_[0], params_.vel2base_[1], params_.vel2base_[2]));
  vel2base = vel2base.inverse();
  t = vel2base.getOrigin();
  vel2base.getBasis().getRPY(roll, pitch, yaw);

  lid_mapper_->setVel2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);

  // Call execution thread
  std::thread th(&LocalizationNode::loop, this);
  th.detach();

  // ROS spin ...
  ROS_INFO("Done! Execution started.");
  ros::spin();
  ROS_INFO("ROS shutting down...");
}

void LocalizationNode::init()
{
  // ---------------------------------------------------------
  // ----- Initialize the localizer and get first particles distribution
  // ---------------------------------------------------------
  localizer_->init(Pose(0, 0, 0, 0, 0, 0));
  robot_pose_ = localizer_->getPose();

  // ---------------------------------------------------------
  // ----- Load the map from the xml input file
  // ---------------------------------------------------------
  MapParser parser(params_);
  parser.parseFile(grid_map_);

  // ---------------------------------------------------------
  // ----- Initialize the multi-layer maps
  // ---------------------------------------------------------

  //  // - 2D semantic feature map
  //  land_mapper_->init(robot_pose_, input_data.land_bearings_, input_data.land_depths_, input_data.land_labels_,
  //                     *grid_map_);
  //
  //  // - 3D PCL corner map estimation
  //  std::vector<Corner> m_corners;
  //  std::vector<Planar> m_planars;
  //  std::vector<Plane> m_planes;
  //  Plane m_ground_plane;
  //  lid_mapper_->localMap(input_data.scan_pts_, m_corners, m_planars, m_planes, m_ground_plane);
  //
  //  // - 3D image feature map estimation
  //  std::vector<ImageFeature> m_surf_features;
  //  vis_mapper_->localMap(input_data.rgb_image_, input_data.depth_array_, m_surf_features);
  //
  //  // - Register 3D maps
  //  vis_mapper_->registerMaps(robot_pose_, m_surf_features, *grid_map_);
  //  lid_mapper_->registerMaps(robot_pose_, m_corners, m_planars, m_planes, *grid_map_);
  //  grid_map_->downsamplePlanars();
  //
  //  // - Save local map for next iteration
  //  previous_map_->clear();
  //  for (const auto& planar : m_planars)
  //    previous_map_->insert(planar);
  //  for (const auto& corner : m_corners)
  //    previous_map_->insert(corner);
  //  previous_map_->downsamplePlanars();
  //
  //  ROS_INFO("Localization has started.");
}

void LocalizationNode::loop()
{
  // Reset information flags
  input_data.received_scans_ = false;
  input_data.received_odometry_ = false;

  while (ros::ok())
  {
    loopOnce();
  }
}

void LocalizationNode::loopOnce()
{
  //  // Check if we have all the necessary data
  //  bool can_continue = input_data.received_scans_ && input_data.received_odometry_;
  //
  //  if (!can_continue)
  //    return;
  //
  // VineSLAM main loop
  if (init_flag_)
  {
    init();
    init_flag_ = false;
  }
  else
    process();

  // Reset information flags
  input_data.received_scans_ = false;
  input_data.received_odometry_ = false;
}

void LocalizationNode::process()
{
  // Publish 3D maps
  publish3DMap();
}

LocalizationNode::~LocalizationNode() = default;

}  // namespace vineslam
