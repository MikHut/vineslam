#include "../include/replay_node.hpp"

int main(int argc, char** argv)
{
  vineslam::ReplayNode replay_node(argc, argv);
  return 0;
}

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Constructor and destructor
// --------------------------------------------------------------------------------

ReplayNode::ReplayNode(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "ReplayNode");
  ros::NodeHandle nh;

  ROS_INFO("Initializing node ...");

  // Load params
  loadParameters(nh, "/replay_node", params_);

  // Set node flags
  nmessages_ = 0;
  bag_state_ = PLAYING;

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

  // Set initialization flags default values
  init_flag_ = true;
  init_gps_ = true;
  init_odom_ = true;
  register_map_ = true;
  estimate_heading_ = true;

  // Services
  polar2pose_ = nh.serviceClient<agrob_map_transform::GetPose>("polar_to_pose");
  set_datum_ = nh.serviceClient<agrob_map_transform::SetDatum>("datum");

  // Publish maps and particle filter
  vineslam_report_publisher_ = nh.advertise<vineslam_msgs::report>("/vineslam/report", 1);
  grid_map_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/occupancyMap", 1);
  elevation_map_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/elevationMap", 1);
  map2D_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map2D", 1);
  map3D_features_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/vineslam/map3D/SURF", 1);
  map3D_corners_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners", 1);
  map3D_planars_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars", 1);
  map3D_planes_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map3D/planes", 1);
  corners_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners_local", 1);
  planars_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars_local", 1);
  planes_local_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map3D/planes_local", 1);
  pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  gps_path_publisher_ = nh.advertise<nav_msgs::Path>("/vineslam/gps_path", 1);
  gps_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/gps_pose", 1);
  path_publisher_ = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher_ = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);

  // Subscribers
  ros::Subscriber gps_subscriber =
      nh.subscribe(params_.fix_topic_, 1, &VineSLAM_ros::gpsListener, dynamic_cast<VineSLAM_ros*>(this));
  ros::Subscriber feat_subscriber = nh.subscribe(params_.image_features_topic_, 1, &VineSLAM_ros::imageFeatureListener,
                                                 dynamic_cast<VineSLAM_ros*>(this));

  // Get image topics
  if (!nh.getParam("/vfe/compressed_image_topic", image_topic_))
  {
    ROS_ERROR("/vfe/compressed_image_topic parameter not found. Shutting down...");
    exit(-1);
  }
  if (!nh.getParam("/vfe/depth_topic", depth_topic_))
  {
    ROS_ERROR("/vfe/depth_topic parameter not found. Shutting down...");
    exit(-1);
  }

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

  // Call execution threads
  ROS_INFO("Done! Execution started.");

  std::thread th1(&ReplayNode::replayFct, this, nh);
  th1.detach();
  std::thread th2(&ReplayNode::listenStdin, this);
  th2.detach();

  ros::spin();
}

void ReplayNode::replayFct(ros::NodeHandle nh)
{
  ROS_INFO("Opening ROSBAG...");
  rosbag::Bag bag;
  bag.open(params_.bagfile_name_);

  // -------------------------------------------------------------------------------
  // ----- Set rosbags topics of interest
  // -------------------------------------------------------------------------------
  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(params_.odom_topic_, 1);
  ros::Publisher tf_pub = nh.advertise<tf2_msgs::TFMessage>(params_.tf_topic_, 1);
  ros::Publisher fix_pub = nh.advertise<sensor_msgs::NavSatFix>(params_.fix_topic_, 1);
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(params_.pcl_topic_, 1);
  ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>(depth_topic_, 1);
  ros::Publisher img_pub = nh.advertise<sensor_msgs::CompressedImage>(image_topic_, 1);

  rosgraph_msgs::Clock clock_ptr;
  tf2_msgs::TFMessageConstPtr tf_ptr;
  nav_msgs::OdometryConstPtr rs_odom_ptr;
  nav_msgs::OdometryConstPtr odom_ptr;
  sensor_msgs::NavSatFixConstPtr fix_ptr;
  sensor_msgs::ImageConstPtr depth_img_ptr;
  cv::Mat left_img;
  sensor_msgs::PointCloud2ConstPtr pcl_ptr;

  ROS_INFO("Reading ROSBAG topics...");

  int num_obsv = 1;  // tf
  if (params_.use_wheel_odometry_)
    num_obsv++;
  if (params_.use_lidar_features_)
    num_obsv++;

  bool tf_bool = false, odom_bool = false, pcl_bool = false;
  for (rosbag::MessageInstance m : rosbag::View(bag))
  {
    while (ros::ok())
    {
      if (bag_state_ == PLAYING || (bag_state_ == ITERATING && !have_iterated_))
        break;
    }

    // Publish clock
    clock_ptr.clock = m.getTime();
    clock_pub.publish(clock_ptr);

    // Publish rosbag topics of interest
    const std::string& topic = m.getTopic();

    if (topic == params_.tf_topic_)
    {
      tf_ptr = m.instantiate<tf2_msgs::TFMessage>();

      if (tf_ptr != nullptr)
      {
        tf_bool = true;
        tf_pub.publish(*tf_ptr);
      }
    }
    else if (topic == params_.odom_topic_)
    {
      odom_ptr = m.instantiate<nav_msgs::Odometry>();

      if (odom_ptr != nullptr)
      {
        odom_bool = true;
        odom_pub.publish(*odom_ptr);
      }
    }
    else if (topic == params_.fix_topic_)
    {
      fix_ptr = m.instantiate<sensor_msgs::NavSatFix>();

      if (fix_ptr != nullptr)
      {
        fix_pub.publish(*fix_ptr);
      }
    }
    else if (topic == params_.pcl_topic_)
    {
      pcl_ptr = m.instantiate<sensor_msgs::PointCloud2>();

      if (pcl_ptr != nullptr)
      {
        pcl_bool = true;
        pcl_pub.publish(*pcl_ptr);
      }
    }
    else if (topic == depth_topic_)
    {
      depth_img_ptr = m.instantiate<sensor_msgs::Image>();

      if (depth_img_ptr != nullptr)
      {
        depth_pub.publish(*depth_img_ptr);
      }
    }
    else if (topic == image_topic_)
    {
      sensor_msgs::CompressedImageConstPtr left_img_comp_ptr = m.instantiate<sensor_msgs::CompressedImage>();

      if (left_img_comp_ptr != nullptr)
      {
        img_pub.publish(*left_img_comp_ptr);
      }
    }


    nmessages_ = tf_bool + (odom_bool && params_.use_wheel_odometry_) + (pcl_bool && params_.use_lidar_features_);

    if (nmessages_ == num_obsv)
    {
      if (params_.use_lidar_features_)
      {
        scanListener(pcl_ptr);
      }
      if (params_.use_wheel_odometry_)
      {
        odomListener(odom_ptr);
      }

      // Call VineSLAM loop
      loopOnce();

      tf_bool = false;
      odom_bool = false;
      pcl_bool = false;
      nmessages_ = 0;
      have_iterated_ = true;
    }
  }

  bag.close();
}

bool ReplayNode::changeNodeState(vineslam_ros::change_replay_node_state::Request& request,
                                 vineslam_ros::change_replay_node_state::Response& response)
{
  if (request.pause_node.data == true)
    bag_state_ = PAUSED;
  if (request.play_node.data == true)
    bag_state_ = PLAYING;
  if (request.iterate_node.data == true)
  {
    have_iterated_ = false;
    bag_state_ = ITERATING;
  }

  return true;
}

void ReplayNode::listenStdin()
{
  std::string input;
  while (ros::ok())
  {
    std::cin >> input;
    if (input == "s" && bag_state_ == PAUSED)
    {
      ROS_INFO("Playing bag file...");
      bag_state_ = PLAYING;
    }
    else if (input == "s" && bag_state_ == PLAYING)
    {
      ROS_INFO("Pausing bag file...");
      bag_state_ = PAUSED;
    }
  }
}

ReplayNode::~ReplayNode() = default;

}  // namespace vineslam
