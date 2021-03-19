#include "../include/slam_node.hpp"

int main(int argc, char** argv)
{
  vineslam::SLAMNode vineslam_node(argc, argv);
  return 0;
}

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Constructor and destructor
// --------------------------------------------------------------------------------

SLAMNode::SLAMNode(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "SLAMNode");
  ros::NodeHandle nh("~");

  // Load params
  loadParameters(nh, "/slam_node", params_);

  //   Set initialization flags default values
  init_flag_ = true;
  init_gps_ = true;
  init_odom_ = true;
  register_map_ = true;

  // Declare the Mappers and Localizer objects
  localizer_ = new Localizer(params_);
  land_mapper_ = new LandmarkMapper(params_);
  vis_mapper_ = new VisualMapper(params_);
  lid_mapper_ = new LidarMapper(params_);
  timer_ = new Timer("VineSLAM subfunctions");

  // Landmark subscription
  ros::Subscriber feat_subscriber =
      nh.subscribe("/features_topic", 1, &VineSLAM_ros::imageFeatureListener, dynamic_cast<VineSLAM_ros*>(this));
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

  // Publish maps and particle filter
  vineslam_report_publisher_ = nh.advertise<vineslam_msgs::report>("/vineslam/report", 1);
  grid_map_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/debug/grid_map_limits", 1);
  elevation_map_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/elevationMap", 1);
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
  path_publisher_ = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher_ = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);

  // ROS services
  ros::ServiceServer start_reg_srv =
      nh.advertiseService("start_registration", &VineSLAM_ros::startRegistration, dynamic_cast<VineSLAM_ros*>(this));
  ros::ServiceServer stop_reg_srv =
      nh.advertiseService("stop_registration", &VineSLAM_ros::stopRegistration, dynamic_cast<VineSLAM_ros*>(this));
  ros::ServiceServer stop_hed_srv = nh.advertiseService(
      "stop_gps_heading_estimation", &VineSLAM_ros::stopHeadingEstimation, dynamic_cast<VineSLAM_ros*>(this));

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

  // Save sensors to map transformation
  // Cam
  tf::Vector3 t_cam = cam2base_msg.getOrigin();
  tfScalar roll_cam, pitch_cam, yaw_cam;
  cam2base_msg.getBasis().getRPY(roll_cam, pitch_cam, yaw_cam);
  vis_mapper_->setCam2Base(t_cam.getX(), t_cam.getY(), t_cam.getZ(), roll_cam, pitch_cam, yaw_cam);
  // LiDAR
  tf::Vector3 t_vel = vel2base_msg.getOrigin();
  tfScalar roll_vel, pitch_vel, yaw_vel;
  vel2base_msg.getBasis().getRPY(roll_vel, pitch_vel, yaw_vel);
  lid_mapper_->setVel2Base(t_vel.getX(), t_vel.getY(), t_vel.getZ(), roll_vel, pitch_vel, yaw_vel);

  // Call execution thread
  std::thread th1(&VineSLAM_ros::loop, dynamic_cast<VineSLAM_ros*>(this));
  std::thread th2(&VineSLAM_ros::publishDenseInfo, dynamic_cast<VineSLAM_ros*>(this));
  th1.detach();
  th2.detach();

  // ROS spin ...
  ROS_INFO("Done! Execution started.");
  ros::spin();
  ROS_INFO("ROS shutting down...");
}

void SLAMNode::loadParameters(const ros::NodeHandle& nh, const std::string& prefix, Parameters& params)
{
  // Load params
  if (!nh.getParam(prefix + "/robot_model", params.robot_model_))
  {
    ROS_WARN("%s/robot_model parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/world_frame_id", params.world_frame_id_))
  {
    ROS_WARN("%s/world_frame_id parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/use_semantic_features", params.use_semantic_features_))
  {
    ROS_WARN("%s/semantic_features parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/use_lidar_features", params.use_lidar_features_))
  {
    ROS_WARN("%s/use_lidar_features parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/use_image_features", params.use_image_features_))
  {
    ROS_WARN("%s/use_image_features parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/use_gps", params.use_gps_))
  {
    ROS_WARN("%s/use_gps parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/use_wheel_odometry", params.use_wheel_odometry_))
  {
    ROS_WARN("%s/use_wheel_odometry parameter has not been set. Not using it...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/camera_info/baseline", params.baseline_))
  {
    ROS_WARN("%s/camera_info/baseline parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/camera_info/fx", params.fx_))
  {
    ROS_WARN("%s/camera_info/fx parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/camera_info/cx", params.cx_))
  {
    ROS_WARN("%s/camera_info/cx parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/origin/x", params.gridmap_origin_x_))
  {
    ROS_WARN("%s/grid_map/origin/x parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/origin/y", params.gridmap_origin_y_))
  {
    ROS_WARN("%s/grid_map/origin/y parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/origin/z", params.gridmap_origin_z_))
  {
    ROS_WARN("%s/grid_map/origin/z parameter not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/width", params.gridmap_width_))
  {
    ROS_WARN("%s/grid_map/width not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/lenght", params.gridmap_lenght_))
  {
    ROS_WARN("%s/grid_map/lenght not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/height", params.gridmap_height_))
  {
    ROS_WARN("%s/grid_map/height not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/resolution", params.gridmap_resolution_))
  {
    ROS_WARN("%s/grid_map/resolution not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/save_map", params.save_map_))
  {
    ROS_WARN("%s/grid_map/save_map not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/output_folder", params.map_output_folder_))
  {
    ROS_WARN("%s/grid_map/output_folder not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/grid_map/input_file", params.map_input_file_))
  {
    ROS_WARN("%s/grid_map/input_file not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/n_particles", params.number_particles_))
  {
    ROS_WARN("%s/pf/n_particles not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_xx", params.sigma_xx_))
  {
    ROS_WARN("%s/pf/sigma_xx not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_yy", params.sigma_yy_))
  {
    ROS_WARN("%s/pf/sigma_yy not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_zz", params.sigma_zz_))
  {
    ROS_WARN("%s/pf/sigma_zz not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_RR", params.sigma_RR_))
  {
    ROS_WARN("%s/pf/sigma_RR not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_PP", params.sigma_PP_))
  {
    ROS_WARN("%s/pf/sigma_pitch not found. ", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/pf/sigma_YY", params.sigma_YY_))
  {
    ROS_WARN("%s/pf/sigma_YY not found. ", prefix.c_str());
  }
}

SLAMNode::~SLAMNode() = default;

}  // namespace vineslam
