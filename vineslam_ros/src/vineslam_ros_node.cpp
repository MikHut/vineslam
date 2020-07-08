#include "../include/vineslam_ros.hpp"

namespace vineslam
{

VineSLAM_ros::VineSLAM_ros(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "SLAMNode");
  ros::NodeHandle nh;

  // Set initialize flag default values
  init = true;

  // Load params
  std::string config_path;
  if (!nh.getParam("/vineslam_ros/SLAMNode/config_path", config_path)) {
    ROS_ERROR("/config_path parameter not found. Shutting down...");
    return;
  }

  // Load config file
  auto config = YAML::LoadFile(config_path);
  // Load camera info parameters
  h_fov      = config["camera_info"]["h_fov"].as<float>() * DEGREE_TO_RAD;
  img_width  = config["camera_info"]["img_width"].as<int>();
  img_height = config["camera_info"]["img_height"].as<int>();
  cam_height = config["camera_info"]["cam_height"].as<float>();
  fx         = config["camera_info"]["fx"].as<float>();
  fy         = config["camera_info"]["fy"].as<float>();
  cx         = config["camera_info"]["cx"].as<float>();
  cy         = config["camera_info"]["cy"].as<float>();
  // Load occupancy grid map parameters
  occ_origin.x   = config["grid_map"]["origin"]["x"].as<float>();
  occ_origin.y   = config["grid_map"]["origin"]["y"].as<float>();
  occ_resolution = config["grid_map"]["resolution"].as<float>();
  occ_width      = config["grid_map"]["width"].as<float>();
  occ_height     = config["grid_map"]["height"].as<float>();
  publish_odom   = config["system"]["publish_odom"].as<bool>();
  use_gps        = config["system"]["use_gps"].as<bool>();
  gps_init_lat   = config["system"]["gps_datum"]["lat"].as<float>();
  gps_init_long  = config["system"]["gps_datum"]["long"].as<float>();
  gps_init_head  = config["system"]["gps_datum"]["head"].as<float>();

  // Declare the Mappers and Localizer objects
  localizer = new Localizer(config_path);
  grid_map  = new OccupancyMap(config_path);
  mapper2D  = new Mapper2D(config_path);
  mapper3D  = new Mapper3D(config_path);

  // Services
  polar2pose = nh.serviceClient<agrob_map_transform::GetPose>("polar_to_pose");
  set_datum  = nh.serviceClient<agrob_map_transform::SetDatum>("datum");

  // Synchronize subscribers of both topics
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(
      nh, "/left_image", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(
      nh, "/depth_image", 1);
  message_filters::Subscriber<vision_msgs::Detection2DArray> detections_sub(
      nh, "/detections", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image,
                                    sensor_msgs::Image,
                                    vision_msgs::Detection2DArray>
      sync(left_image_sub, depth_image_sub, detections_sub, 10);
  sync.registerCallback(boost::bind(&VineSLAM_ros::callbackFct, this, _1, _2, _3));

  // Odometry subscription
  ros::Subscriber odom_subscriber =
      nh.subscribe("/odom", 1, &VineSLAM_ros::odomListener, this);
  ros::Subscriber gps_subscriber =
      nh.subscribe("/fix", 1, &VineSLAM_ros::gpsListener, this);

  // Publish maps and particle filter
  mapOCC_publisher =
      nh.advertise<nav_msgs::OccupancyGrid>("/vineslam/occupancyMap", 1);
  map2D_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map2D", 1);
  map3D_features_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/vineslam/map3D/SURF", 1);
  map3D_corners_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners", 1);
  map3D_planes_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/ground", 1);
  map3D_debug_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/debug", 1);
  normal_pub = nh.advertise<visualization_msgs::Marker>("/map3D/ground_normal", 1);
  pose_publisher  = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  odom_publisher  = nh.advertise<nav_msgs::Odometry>("/vineslam/odom", 1);
  gps_publisher   = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/gps", 1);
  path_publisher  = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);

  // GNSS varibales
  if (use_gps) {
    datum_autocorrection_stage = 0;
    global_counter             = 0;
  }

  ros::spin();
  ROS_INFO("ROS shutting down...");
}

} // namespace vineslam
