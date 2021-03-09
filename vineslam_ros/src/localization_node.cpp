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
  // ---------------------------------------------------------
  // Initialize ROS node
  // ---------------------------------------------------------
  ros::init(argc, argv, "LocalizationNode");
  ros::NodeHandle nh;

  // ---------------------------------------------------------
  // ----- Load params
  // ---------------------------------------------------------
  loadParameters(nh, "/localization_node", params_);

  // ---------------------------------------------------------
  // ----- Set initialization flags default values
  // ---------------------------------------------------------
  init_flag_ = true;
  init_gps_ = true;
  init_odom_ = true;
  register_map_ = false;
  estimate_heading_ = true;

  // ---------------------------------------------------------
  // ----- Declare the Mappers and Localizer objects
  // ---------------------------------------------------------
  localizer_ = new Localizer(params_);
  land_mapper_ = new LandmarkMapper(params_);
  vis_mapper_ = new VisualMapper(params_);
  lid_mapper_ = new LidarMapper(params_);

  // ---------------------------------------------------------
  // ----- Initialize local grid map that will be used for relative motion calculation
  // ---------------------------------------------------------
  Parameters local_map_params;
  local_map_params.gridmap_origin_x_ = -30;
  local_map_params.gridmap_origin_y_ = -30;
  local_map_params.gridmap_origin_z_ = -0.5;
  local_map_params.gridmap_resolution_ = 0.20;
  local_map_params.gridmap_width_ = 60;
  local_map_params.gridmap_lenght_ = 60;
  local_map_params.gridmap_height_ = 2.5;
  previous_map_ = new OccupancyMap(local_map_params, Pose(0, 0, 0, 0, 0, 0));

  // ---------------------------------------------------------
  // ----- Services
  // ---------------------------------------------------------
  polar2pose_ = nh.serviceClient<agrob_map_transform::GetPose>("polar_to_pose");
  set_datum_ = nh.serviceClient<agrob_map_transform::SetDatum>("datum");

 // Landmark subscription
  ros::Subscriber feat_subscriber = nh.subscribe("/features_topic", 1, &VineSLAM_ros::imageFeatureListener,
                                                 dynamic_cast<VineSLAM_ros*>(this));
  ros::Subscriber land_subscriber =
      nh.subscribe("/detections_topic", 1, &VineSLAM_ros::landmarkListener, dynamic_cast<VineSLAM_ros*>(this));
  // Scan subscription
  ros::Subscriber scan_subscriber =
      nh.subscribe("/scan_topic", 1, &VineSLAM_ros::scanListener, dynamic_cast<VineSLAM_ros*>(this));
  // Odometry subscription
  ros::Subscriber odom_subscriber =
      nh.subscribe("/odom_topic", 1, &VineSLAM_ros::odomListener, dynamic_cast<VineSLAM_ros*>(this));
  // GPS subscription
  ros::Subscriber gps_subscriber =
      nh.subscribe("/gps_topic", 1, &VineSLAM_ros::gpsListener, dynamic_cast<VineSLAM_ros*>(this));

  // ---------------------------------------------------------
  // ----- Publish maps and particle filter
  // ---------------------------------------------------------
  vineslam_report_publisher_ = nh.advertise<vineslam_msgs::report>("/vineslam/report", 1);
  grid_map_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/occupancyMap", 1);
  map2D_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map2D", 1);
  map3D_features_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/vineslam/map3D/SURF", 1);
  map3D_corners_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners", 1);
  map3D_planars_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars", 1);
  map3D_planes_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map3D/planes", 1);
  planes_local_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map3D/planes_local", 1);
  corners_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners_local", 1);
  planars_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars_local", 1);
  pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  odom_publisher_ = nh.advertise<nav_msgs::Odometry>("/vineslam/odom", 1);
  gps_path_publisher_ = nh.advertise<nav_msgs::Path>("/vineslam/gps_path", 1);
  gps_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/gps_pose", 1);
  path_publisher_ = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher_ = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);

  // ---------------------------------------------------------
  // ----- ROS services
  // ---------------------------------------------------------
  ros::ServiceServer stop_hed_srv = nh.advertiseService(
      "stop_gps_heading_estimation", &VineSLAM_ros::stopHeadingEstimation, dynamic_cast<VineSLAM_ros*>(this));

  // ---------------------------------------------------------
  // ----- GNSS varibales
  // ---------------------------------------------------------
  if (params_.use_gps_)
  {
    datum_autocorrection_stage_ = 0;
    global_counter_ = 0;
  }

  ROS_INFO("Loading map from xml file...");
  // ---------------------------------------------------------
  // ----- Load map dimensions and initialize it
  // ---------------------------------------------------------
  MapParser parser(params_);
  parser.parseHeader(&params_);
  grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0));

  // ---------------------------------------------------------
  // ----- Load the map from the xml input file
  // ---------------------------------------------------------
  parser.parseFile(&(*grid_map_));
  ROS_INFO("The map has been loaded...");

  ROS_INFO("Allocating map memory...");
  grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0));
  elevation_map_ = new ElevationMap(params_, Pose(0, 0, 0, 0, 0, 0));

  ROS_INFO("Waiting for static transforms...");
  tf::TransformListener listener;
  tf::StampedTransform cam2base_msg, vel2base_msg;
  bool got_cam2base = false, got_vel2base = false;
  while (!got_cam2base && ros::ok())
  {
    try
    {
      listener.lookupTransform("/zed_camera_left_optical_frame", "/base_link", ros::Time(0), cam2base_msg);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.5).sleep();
      continue;
    }
    got_cam2base = true;
  }
  while (!got_vel2base && ros::ok())
  {
    try
    {
      listener.lookupTransform("/velodyne", "/base_link", ros::Time(0), vel2base_msg);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.5).sleep();
      continue;
    }
    got_vel2base = true;
  }
  ROS_INFO("Received!");

  // ---------------------------------------------------------
  // ----- Call execution thread
  // ---------------------------------------------------------
  std::thread th(&VineSLAM_ros::loop, dynamic_cast<VineSLAM_ros*>(this));
  th.detach();

  // ---------------------------------------------------------
  // ----- ROS spin ...
  // ---------------------------------------------------------
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
    //    vis_mapper_->localMap(input_data_.rgb_image_, input_data_.depth_array_, l_surf_features);
    vis_mapper_->localMap(input_data_.image_features_, l_surf_features);
  }

  if (register_map_)
  {
    // - Register 3D maps
    vis_mapper_->registerMaps(robot_pose_, l_surf_features, *grid_map_);
    lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, l_ground_plane, *grid_map_);
    grid_map_->downsamplePlanars();
  }

  ROS_INFO("Localization and Mapping has started.");
}

void LocalizationNode::loadParameters(const ros::NodeHandle& nh, const std::string& prefix, Parameters& params)
{
  const std::string& node_name = ros::this_node::getName();

  // Load params
  if (!nh.getParam(prefix + "/use_semantic_features", params.use_semantic_features_))
  {
    ROS_WARN("%s/semantic_features parameter not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/use_lidar_features", params.use_lidar_features_))
  {
    ROS_WARN("%s/use_lidar_features parameter not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/use_image_features", params.use_image_features_))
  {
    ROS_WARN("%s/use_image_features parameter not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/use_gps", params.use_gps_))
  {
    ROS_WARN("%s/use_gps parameter not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/use_wheel_odometry", params.use_wheel_odometry_))
  {
    ROS_WARN("%s/use_wheel_odometry parameter has not been set. Not using it...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/gps_datum/lat", params.latitude_))
  {
    ROS_WARN("%s/gps_datum/lat parameter not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/gps_datum/long", params.longitude_))
  {
    ROS_WARN("%s/gps_datum/long parameter not found. Shutting down...", prefix.c_str());
  }
 if (!nh.getParam(prefix + "/camera_info/baseline", params.baseline_))
  {
    ROS_WARN("%s/camera_info/baseline parameter not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/camera_info/fx", params.fx_))
  {
    ROS_WARN("%s/camera_info/fx parameter not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/camera_info/cx", params.cx_))
  {
    ROS_WARN("%s/camera_info/cx parameter not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/input_file", params.map_input_file_))
  {
    ROS_WARN("%s/grid_map/input_file not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/n_particles", params.number_particles_))
  {
    ROS_WARN("%s/pf/n_particles not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_xx", params.sigma_xx_))
  {
    ROS_WARN("%s/pf/sigma_xx not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_yy", params.sigma_yy_))
  {
    ROS_WARN("%s/pf/sigma_yy not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_zz", params.sigma_zz_))
  {
    ROS_WARN("%s/pf/sigma_zz not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_RR", params.sigma_RR_))
  {
    ROS_WARN("%s/pf/sigma_RR not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_PP", params.sigma_PP_))
  {
    ROS_WARN("%s/pf/sigma_pitch not found. Shutting down...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_YY", params.sigma_YY_))
  {
    ROS_WARN("%s/pf/sigma_YY not found. Shutting down...", prefix.c_str());
  }
}

LocalizationNode::~LocalizationNode() = default;

}  // namespace vineslam
