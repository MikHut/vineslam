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
  ros::NodeHandle nh;

  // Load params
  loadParameters(nh, "/slam_node", params_);

  // Set initialization flags default values
  init_flag_ = true;
  init_gps_ = true;
  init_odom_ = true;
  register_map_ = true;
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
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh, params_.rgb_img_topic_, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh, params_.depth_img_topic_, 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(left_image_sub, depth_image_sub, 10);
  sync.registerCallback(boost::bind(&SLAMNode::imageListener, this, _1, _2));

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
  gps_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/gps", 1);
  path_publisher_ = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher_ = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);

  // ROS services
  ros::ServiceServer start_reg_srv =
      nh.advertiseService("start_registration", &VineSLAM_ros::startRegistration, dynamic_cast<VineSLAM_ros*>(this));
  ros::ServiceServer stop_reg_srv =
      nh.advertiseService("stop_registration", &VineSLAM_ros::stopRegistration, dynamic_cast<VineSLAM_ros*>(this));
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
  std::thread th(&VineSLAM_ros::loop, dynamic_cast<VineSLAM_ros*>(this));
  th.detach();

  // ROS spin ...
  ROS_INFO("Done! Execution started.");
  ros::spin();
  ROS_INFO("ROS shutting down...");
}

SLAMNode::~SLAMNode() = default;

}  // namespace vineslam
