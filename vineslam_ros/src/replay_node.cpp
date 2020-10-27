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
  loadParameters(nh, "/replay_node", params);

  // Set node flags
  nmessages = 0;
  bag_state = PLAYING;

  // Declare the Mappers and Localizer objects
  localizer = new Localizer(params);
  grid_map  = new OccupancyMap(params);
  mapper2D  = new Mapper2D(params);
  mapper3D  = new Mapper3D(params);


  // Set initialization flags default values
  init             = true;
  init_gps         = true;
  init_odom        = true;
  register_map     = true;
  estimate_heading = true;

  // Services
  polar2pose = nh.serviceClient<agrob_map_transform::GetPose>("polar_to_pose");
  set_datum  = nh.serviceClient<agrob_map_transform::SetDatum>("datum");

  // Publish maps and particle filter
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
  corners_local_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "/vineslam/map3D/corners_local", 1);
  planars_local_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "/vineslam/map3D/planars_local", 1);
  planes_local_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "/vineslam/map3D/planes_local", 1);
  pose_publisher  = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  gps_publisher   = nh.advertise<nav_msgs::Path>("/vineslam/gps", 1);
  path_publisher  = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);
  // Debug publishers
  debug_markers =
      nh.advertise<visualization_msgs::MarkerArray>("/vineslam/debug_markers", 1);

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

    mapper3D->setCam2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);
    mapper2D->setCamPitch(pitch);

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

    mapper3D->setVel2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);
  }

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
  bag.open(params.bagfile_name);

  // -------------------------------------------------------------------------------
  // ----- Set static TFs
  // -------------------------------------------------------------------------------
  tf::Transform unit_tf;
  unit_tf.setRotation(tf::Quaternion(0., 0., 0., 1.));
  unit_tf.setOrigin(tf::Vector3(0., 0., 0.));

  // -------------------------------------------------------------------------------
  // ----- Set rosbags topics of interest
  // -------------------------------------------------------------------------------
  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
  ros::Publisher odom_pub  = nh.advertise<nav_msgs::Odometry>(params.odom_topic, 1);
  ros::Publisher tf_pub  = nh.advertise<tf2_msgs::TFMessage>(params.tf_topic, 1);
  ros::Publisher fix_pub = nh.advertise<sensor_msgs::NavSatFix>(params.fix_topic, 1);
  ros::Publisher depth_img_pub =
      nh.advertise<sensor_msgs::Image>(params.depth_img_topic, 1);
  ros::Publisher left_img_pub =
      nh.advertise<sensor_msgs::CompressedImage>(params.left_img_topic, 1);
  ros::Publisher pcl_pub =
      nh.advertise<sensor_msgs::PointCloud2>(params.pcl_topic, 1);

  rosgraph_msgs::Clock             clock_ptr;
  tf2_msgs::TFMessageConstPtr      tf_ptr;
  nav_msgs::OdometryConstPtr       rs_odom_ptr;
  nav_msgs::OdometryConstPtr       odom_ptr;
  sensor_msgs::NavSatFixConstPtr   fix_ptr;
  sensor_msgs::ImageConstPtr       depth_img_ptr;
  cv::Mat                          left_img;
  sensor_msgs::PointCloud2ConstPtr pcl_ptr;

  ROS_INFO("Reading ROSBAG topics...");
  bool tf_bool = false, clock_bool = false, odom_bool = false, fix_bool = false,
       depth_bool = false, left_bool = false, pcl_bool = false;
  for (rosbag::MessageInstance m : rosbag::View(bag)) {
    while (ros::ok()) {
      if (bag_state == PLAYING)
        break;
    }

    // Publish clock
    clock_ptr.clock = m.getTime();
    clock_pub.publish(clock_ptr);

    // Publish rosbag topics of interest
    const std::string& topic = m.getTopic();

    if (topic == params.tf_topic) {
      tf_ptr = m.instantiate<tf2_msgs::TFMessage>();

      if (tf_ptr != nullptr) {
        tf_bool = true;
        tf_pub.publish(*tf_ptr);
      }
    } else if (topic == params.odom_topic) {
      odom_ptr = m.instantiate<nav_msgs::Odometry>();

      if (odom_ptr != nullptr) {
        odom_bool = true;
        odom_pub.publish(*odom_ptr);
      }
    } else if (topic == params.fix_topic) {
      fix_ptr = m.instantiate<sensor_msgs::NavSatFix>();

      if (fix_ptr != nullptr) {
        fix_bool = true;
        fix_pub.publish(*fix_ptr);
      }
    } else if (topic == params.depth_img_topic) {
      depth_img_ptr = m.instantiate<sensor_msgs::Image>();

      if (depth_img_ptr != nullptr) {
        depth_bool = true;
        depth_img_pub.publish(*depth_img_ptr);
      }
    } else if (topic == params.left_img_topic) {
      sensor_msgs::CompressedImageConstPtr left_img_comp_ptr =
          m.instantiate<sensor_msgs::CompressedImage>();

      if (left_img_comp_ptr != nullptr) {
        left_bool = true;
        left_img_pub.publish(*left_img_comp_ptr);

        // Decompress image
        cv::Mat img_data(1, left_img_comp_ptr->data.size(), CV_8UC3);
        img_data.data = const_cast<uchar*>(&left_img_comp_ptr->data[0]);
        cv::InputArray data(img_data);
        left_img = cv::imdecode(data, 1);
      }
    } else if (topic == params.pcl_topic) {
      pcl_ptr = m.instantiate<sensor_msgs::PointCloud2>();

      if (pcl_ptr != nullptr) {
        pcl_bool = true;
        pcl_pub.publish(*pcl_ptr);
      }
    }

    nmessages = tf_bool + odom_bool + fix_bool + depth_bool + left_bool + pcl_bool;

    if (nmessages == 6) {
      scanListener(pcl_ptr);
      odomListener(odom_ptr);
      // TODO (Andre Aguiar): GPS should be supported in the future
      // gpsListener(fix_ptr);
      mainFct(left_img, depth_img_ptr, nullptr);

      tf_bool    = false;
      odom_bool  = false;
      fix_bool   = false;
      depth_bool = false;
      left_bool  = false;
      pcl_bool   = false;
      nmessages  = 0;
    }
  }

  bag.close();
}

void ReplayNode::listenStdin()
{
  std::string input;
  while (ros::ok()) {
    std::cin >> input;
    if (input == "s" && bag_state == PAUSED) {
      ROS_INFO("Playing bag file...");
      bag_state = PLAYING;
    } else if (input == "s" && bag_state == PLAYING) {
      ROS_INFO("Pausing bag file...");
      bag_state = PAUSED;
    }
  }
}

ReplayNode::~ReplayNode()
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
