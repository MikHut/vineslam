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
  init             = true;
  init_gps         = true;
  init_odom        = true;
  register_map     = true;
  estimate_heading = true;

  // Declare the Mappers and Localizer objects
  localizer = new Localizer(params);
  grid_map  = new OccupancyMap(params);
  mapper2D  = new Mapper2D(params);
  mapper3D  = new Mapper3D(params);

  // Services
  polar2pose = nh.serviceClient<agrob_map_transform::GetPose>("polar_to_pose");
  set_datum  = nh.serviceClient<agrob_map_transform::SetDatum>("datum");

  // Synchronize subscribers of both topics
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(
      nh, params.left_img_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(
      nh, params.depth_img_topic, 1);
  message_filters::Subscriber<vision_msgs::Detection2DArray> detections_sub(
      nh, params.detections_topic, 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image,
                                    sensor_msgs::Image,
                                    vision_msgs::Detection2DArray>
      sync(left_image_sub, depth_image_sub, detections_sub, 10);
  sync.registerCallback(boost::bind(&SLAMNode::mainCallbackFct, this, _1, _2, _3));

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
  mapOCC_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("/vineslam/occupancyMap", 1);
  map2D_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map2D", 1);
  map3D_features_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/vineslam/map3D/SURF", 1);
  map3D_corners_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners", 1);
  map3D_planes_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/ground", 1);
  map3D_lines_publisher =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/lines", 1);
  pose_publisher  = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  gps_publisher   = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/gps", 1);
  path_publisher  = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);
  // Debug publishers
  corners_local_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "/vineslam/map3D/corners_local", 1);
  debug_markers =
      nh.advertise<visualization_msgs::MarkerArray>("/vineslam/debug_markers", 1);
  exec_boolean = nh.advertise<std_msgs::Bool>("/vineslam/execution_bool", 1);

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
  bool got_tfs = false;
  while (!got_tfs) {
    got_tfs = true;

    tf::Transform cam2base;
    cam2base.setRotation(tf::Quaternion(-0.002, 0.100, -0.004, 0.995));
    //    cam2base.setRotation(tf::Quaternion(0, 0.1691, 0, 0.9856));
    cam2base.setOrigin(tf::Vector3(0.343, 0.079, 0.820));
    cam2base      = cam2base.inverse();
    tf::Vector3 t = cam2base.getOrigin();
    tfScalar    roll, pitch, yaw;
    cam2base.getBasis().getRPY(roll, pitch, yaw);

    mapper3D->setCam2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);
    mapper2D->setCamPitch(pitch);

    tf::Transform vel2base;
    vel2base.setRotation(tf::Quaternion(-0.010, 0.002, -0.045, 0.999));
    vel2base.setOrigin(tf::Vector3(0.286, 0.025, 0.920));
    vel2base = vel2base.inverse();
    t        = vel2base.getOrigin();
    vel2base.getBasis().getRPY(roll, pitch, yaw);

    mapper3D->setVel2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);
  }

  ROS_INFO("Got the transforms! Initializing maps...");

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

  // Save path data
  saveRobotPathKitti(gps_path, odom_path, robot_path);
}

} // namespace vineslam
