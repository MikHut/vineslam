#include "../include/wildSLAM_ros.hpp"

wildSLAM_ros::SLAMNode::SLAMNode(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "SLAMNode");
  ros::NodeHandle nh;

  // Set initialize flag default values
  init = true;

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
  sync.registerCallback(boost::bind(&SLAMNode::callbackFct, this, _1, _2, _3));

  // Odometry subscription
  ros::Subscriber odom_subscriber =
      nh.subscribe("/odom", 1, &SLAMNode::odomListener, this);

  // Publish maps and particle filter
  mapOcc_publisher =
      nh.advertise<nav_msgs::OccupancyGrid>("/wildSLAM/occupancyMap", 1);
  map2D_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("/wildSLAM/map2D", 1);
  map3D_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("/wildSLAM/map3D", 1);
  pose_publisher  = nh.advertise<geometry_msgs::PoseStamped>("/wildSLAM/pose", 1);
  path_publisher  = nh.advertise<nav_msgs::Path>("/wildSLAM/path", 1);
  poses_publisher = nh.advertise<geometry_msgs::PoseArray>("/wildSLAM/poses", 1);

  // Load params
  std::string config_path;
  if (!nh.getParam("/wildSLAM_ros/SLAMNode/config_path", config_path)) {
    ROS_ERROR("/config_path parameter not found. Shutting down...");
    return;
  }

  // Load config file
  auto config = YAML::LoadFile(config_path);
  // Load camera info parameters
  h_fov      = config["camera_info"]["h_fov"].as<float>() * PI / 180;
  img_width  = config["camera_info"]["img_width"].as<float>();
  img_height = config["camera_info"]["img_height"].as<float>();
  cam_height = config["camera_info"]["cam_height"].as<float>();
  fx         = config["camera_info"]["fx"].as<float>();
  fy         = config["camera_info"]["fy"].as<float>();
  cx         = config["camera_info"]["cx"].as<float>();
  cy         = config["camera_info"]["cy"].as<float>();
  // Load 3D octomap parameters
  res        = config["mapper3D"]["resolution"].as<float>();
  prob_hit   = config["mapper3D"]["hit"].as<float>();
  prob_miss  = config["mapper3D"]["miss"].as<float>();
  thresh_min = config["mapper3D"]["thresh_min"].as<float>();
  thresh_max = config["mapper3D"]["thresh_max"].as<float>();
  max_range  = config["mapper3D"]["max_range"].as<float>();
  // Load occupancy grid map parameters
  occ_origin.x   = config["grid_map"]["origin"]["x"].as<float>();
  occ_origin.y   = config["grid_map"]["origin"]["y"].as<float>();
  occ_resolution = config["grid_map"]["resolution"].as<float>();
  occ_width      = config["grid_map"]["width"].as<float>();
  occ_height     = config["grid_map"]["height"].as<float>();

  // Declare the Mappers and Localizer objects
  localizer = new Localizer(config_path);
  grid_map  = new OccupancyMap(config_path);
  mapper2D  = new Mapper2D(config_path);
  mapper3D  = new Mapper3D(config_path);

  // Initialize 3D octomap(s)
  feature_octree = new OcTreeT(res);
  (*feature_octree).setProbHit(prob_hit);
  (*feature_octree).setProbMiss(prob_miss);
  (*feature_octree).setClampingThresMin(thresh_min);
  (*feature_octree).setClampingThresMax(thresh_max);

  ros::spin();
  ROS_INFO("ROS shutting down...");
}
