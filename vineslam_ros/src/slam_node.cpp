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
  loadParameters(nh, "/slam_node", params);

  // Set initialization flags default values
  init_flag        = true;
  init_gps         = true;
  init_odom        = true;
  register_map     = true;
  estimate_heading = true;

  // Declare the Mappers and Localizer objects
  localizer   = new Localizer(params);
  land_mapper = new LandmarkMapper(params);
  vis_mapper  = new VisualMapper(params);
  lid_mapper  = new LidarMapper(params);

  // Initialize local grid map that will be used for relative motion calculation
  Parameters local_map_params;
  local_map_params.gridmap_origin_x   = -30;
  local_map_params.gridmap_origin_y   = -30;
  local_map_params.gridmap_origin_z   = -0.5;
  local_map_params.gridmap_resolution = 0.20;
  local_map_params.gridmap_width      = 60;
  local_map_params.gridmap_lenght     = 60;
  local_map_params.gridmap_height     = 2.5;
  previous_map = new OccupancyMap(local_map_params, pose(0, 0, 0, 0, 0, 0));

  // Services
  polar2pose = nh.serviceClient<agrob_map_transform::GetPose>("polar_to_pose");
  set_datum  = nh.serviceClient<agrob_map_transform::SetDatum>("datum");

  // Synchronize subscribers of both image topics
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(
      nh, params.left_img_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(
      nh, params.depth_img_topic, 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(
      left_image_sub, depth_image_sub, 10);
  sync.registerCallback(boost::bind(&SLAMNode::imageListener, this, _1, _2));

  // Landmark subscription
  ros::Subscriber land_subscriber = nh.subscribe(params.detections_topic,
                                                 1,
                                                 &VineSLAM_ros::landmarkListener,
                                                 dynamic_cast<VineSLAM_ros*>(this));
  // Scan subscription
  ros::Subscriber scan_subscriber = nh.subscribe(params.pcl_topic,
                                                 1,
                                                 &VineSLAM_ros::scanListener,
                                                 dynamic_cast<VineSLAM_ros*>(this));
  // Odometry subscription
  ros::Subscriber odom_subscriber = nh.subscribe(params.odom_topic,
                                                 1,
                                                 &VineSLAM_ros::odomListener,
                                                 dynamic_cast<VineSLAM_ros*>(this));
  // GPS subscription
  ros::Subscriber gps_subscriber = nh.subscribe(params.fix_topic,
                                                1,
                                                &VineSLAM_ros::gpsListener,
                                                dynamic_cast<VineSLAM_ros*>(this));

  // Publish maps and particle filter
  vineslam_report_publisher =
      nh.advertise<vineslam_msgs::report>("/vineslam/report", 1);
  grid_map_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("/vineslam/occupancyMap", 1);
  map2D_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map2D", 1);
  map3D_features_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/vineslam/map3D/SURF", 1);
  map3D_corners_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners", 1);
  map3D_planars_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars", 1);
  planes_local_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "/vineslam/map3D/planes_local", 1);
  corners_local_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "/vineslam/map3D/corners_local", 1);
  planars_local_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "/vineslam/map3D/planars_local", 1);
  pose_publisher  = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  gps_publisher   = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/gps", 1);
  path_publisher  = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);

  // ROS services
  ros::ServiceServer start_reg_srv =
      nh.advertiseService("start_registration",
                          &VineSLAM_ros::startRegistration,
                          dynamic_cast<VineSLAM_ros*>(this));
  ros::ServiceServer stop_reg_srv =
      nh.advertiseService("stop_registration",
                          &VineSLAM_ros::stopRegistration,
                          dynamic_cast<VineSLAM_ros*>(this));
  ros::ServiceServer stop_hed_srv =
      nh.advertiseService("stop_gps_heading_estimation",
                          &VineSLAM_ros::stopHeadingEstimation,
                          dynamic_cast<VineSLAM_ros*>(this));

  // GNSS varibales
  if (params.use_gps) {
    datum_autocorrection_stage = 0;
    global_counter             = 0;
  }

  // Get static sensor tfs
  tf::Transform cam2base;
  cam2base.setRotation(tf::Quaternion(params.cam2base[3],
                                      params.cam2base[4],
                                      params.cam2base[5],
                                      params.cam2base[6]));
  cam2base.setOrigin(
      tf::Vector3(params.cam2base[0], params.cam2base[1], params.cam2base[2]));
  cam2base      = cam2base.inverse();
  tf::Vector3 t = cam2base.getOrigin();
  tfScalar    roll, pitch, yaw;
  cam2base.getBasis().getRPY(roll, pitch, yaw);

  vis_mapper->setCam2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);
  land_mapper->setCamPitch(pitch);

  tf::Transform vel2base;
  vel2base.setRotation(tf::Quaternion(params.vel2base[3],
                                      params.vel2base[4],
                                      params.vel2base[5],
                                      params.vel2base[6]));
  vel2base.setOrigin(
      tf::Vector3(params.vel2base[0], params.vel2base[1], params.vel2base[2]));
  vel2base = vel2base.inverse();
  t        = vel2base.getOrigin();
  vel2base.getBasis().getRPY(roll, pitch, yaw);

  lid_mapper->setVel2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);

  // Call execution thread
  std::thread th(&VineSLAM_ros::loop, dynamic_cast<VineSLAM_ros*>(this));
  th.detach();

  // ROS spin ...
  ROS_INFO("Done! Execution started.");

  ros::spin();
  ROS_INFO("ROS shutting down...");
}

SLAMNode::~SLAMNode()
{
  // Save map data
  bool save_map = params.save_map;

  if (save_map) {
    std::cout << "Writing map to file ..." << std::endl;
    MapWriter mw(params);
    mw.writeToFile(*grid_map);
  }
}

} // namespace vineslam
