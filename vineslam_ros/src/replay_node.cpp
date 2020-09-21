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

  // Load params
  loadParameters(nh, "/replay_node", params);

  // Set node flags
  nmessages = 0;

  std::thread th(&ReplayNode::replayFct, this, nh);
  th.detach();

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
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planes", 1);
  pose_publisher  = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  gps_publisher   = nh.advertise<nav_msgs::Path>("/vineslam/gps", 1);
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
    vel2base.setRotation(tf::Quaternion(0., 0., 0., 1.));
    vel2base.setOrigin(tf::Vector3(0., 0., 0.942));
    vel2base = vel2base.inverse();
    t        = vel2base.getOrigin();
    vel2base.getBasis().getRPY(roll, pitch, yaw);

    mapper3D->setVel2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);
  }

  ROS_INFO("Got the transforms! Initializing maps...");

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
  ros::Publisher rs_odom_pub =
      nh.advertise<nav_msgs::Odometry>(params.rs_odom_topic, 1);
  ros::Publisher tf_pub  = nh.advertise<tf2_msgs::TFMessage>(params.tf_topic, 1);
  ros::Publisher fix_pub = nh.advertise<sensor_msgs::NavSatFix>(params.fix_topic, 1);
  ros::Publisher depth_img_pub =
      nh.advertise<sensor_msgs::Image>(params.depth_img_topic, 1);
  ros::Publisher left_img_pub =
      nh.advertise<sensor_msgs::CompressedImage>(params.left_img_topic, 1);
  ros::Publisher pcl_pub =
      nh.advertise<sensor_msgs::PointCloud2>(params.pcl_topic, 1);

  rosgraph_msgs::ClockConstPtr     clock_ptr;
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
  for (rosbag::MessageInstance const m : rosbag::View(bag)) {
    // Publish clock
    clock_pub.publish(m.getTime());

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
//      gpsListener(fix_ptr);
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
