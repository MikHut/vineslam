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
  mapper2D  = new Mapper2D(params);
  mapper3D  = new Mapper3D(params);
  pf        = new PF(params, pose(0, 0, 0, 0, 0, 0));

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
  debug_pf_particles_pub =
      nh.advertise<geometry_msgs::PoseArray>("/vineslam/debug/poses", 1);
  debug_pf_weights_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/vineslam/debug/weights", 1);
  debug_pf_corners_local_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "/vineslam/debug/map3D/corners_local", 1);
  debug_pf_planars_local_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "/vineslam/debug/map3D/planars_local", 1);
  debug_pf_planes_local_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "/vineslam/debug/map3D/planes_local", 1);

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
  ros::ServiceServer change_node_state_srv = nh.advertiseService(
      "change_replay_node_state", &ReplayNode::changeNodeState, this);
  ros::ServiceServer change_node_features_srv = nh.advertiseService(
      "change_replay_node_features", &ReplayNode::changeNodeFeatures, this);
  ros::ServiceServer debug_pf_srv =
      nh.advertiseService("debug_pf", &ReplayNode::debugPF, this);

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
  // ----- Set rosbags topics of interest
  // -------------------------------------------------------------------------------
  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
  ros::Publisher odom_pub  = nh.advertise<nav_msgs::Odometry>(params.odom_topic, 1);
  ros::Publisher tf_pub    = nh.advertise<tf2_msgs::TFMessage>(params.tf_topic, 1);
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

  bool tf_bool = false, odom_bool = false, fix_bool = false, depth_bool = false,
       left_bool = false, pcl_bool = false;
  for (rosbag::MessageInstance m : rosbag::View(bag)) {
    while (ros::ok()) {
      if (bag_state == PLAYING || (bag_state == ITERATING && !have_iterated))
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
      // Save data to use on debug procedure
      if (!init) {
        m_particles.clear();
        localizer->getParticles(m_particles);
        m_grid_map = grid_map;
      }

      scanListener(pcl_ptr);
      odomListener(odom_ptr);
      // TODO (Andre Aguiar): GPS should be supported in the future
      // gpsListener(fix_ptr);
      mainFct(left_img, depth_img_ptr, nullptr);

      tf_bool       = false;
      odom_bool     = false;
      fix_bool      = false;
      depth_bool    = false;
      left_bool     = false;
      pcl_bool      = false;
      nmessages     = 0;
      have_iterated = true;
    }
  }

  bag.close();
}

bool ReplayNode::changeNodeState(
    vineslam_ros::change_replay_node_state::Request&  request,
    vineslam_ros::change_replay_node_state::Response& response)
{
  if (request.pause_node.data == true)
    bag_state = PAUSED;
  if (request.play_node.data == true)
    bag_state = PLAYING;
  if (request.iterate_node.data == true) {
    have_iterated = false;
    bag_state     = ITERATING;
  }

  return true;
}

bool ReplayNode::changeNodeFeatures(
    vineslam_ros::change_replay_node_features::Request&  request,
    vineslam_ros::change_replay_node_features::Response& response)
{
  params.use_landmarks    = request.use_high_level.data;
  params.use_corners      = request.use_corners.data;
  params.use_planars      = request.use_planars.data;
  params.use_planes       = request.use_planes.data;
  params.use_ground_plane = request.use_ground.data;
  params.use_icp          = request.use_icp.data;
  params.use_gps          = request.use_gps.data;

  localizer->changeObservationsToUse(params.use_landmarks,
                                     params.use_corners,
                                     params.use_planars,
                                     params.use_planes,
                                     params.use_ground_plane,
                                     params.use_icp,
                                     params.use_gps);

  return true;
}

bool ReplayNode::debugPF(vineslam_ros::debug_particle_filter::Request&  request,
                         vineslam_ros::debug_particle_filter::Response& response)
{
  ROS_INFO("Particle filter debugging procedure has started ...");

  // -------------------------------------------------------------------------------
  // ---- Create set of particles from the limits established by the user
  // -------------------------------------------------------------------------------
  float w = 1 / static_cast<float>(m_particles.size());
  for (size_t i = 0; i < m_particles.size(); i++) {
    pose p_pose = m_particles[i].p;
    pose m_pose =
        m_particles[i].p + pose(sampleGaussian(request.x_std),
                                sampleGaussian(request.y_std),
                                sampleGaussian(request.z_std),
                                sampleGaussian(request.R_std * DEGREE_TO_RAD),
                                sampleGaussian(request.P_std * DEGREE_TO_RAD),
                                sampleGaussian(request.Y_std * DEGREE_TO_RAD));

    pf->particles[i]    = Particle(i, m_pose, w);
    pf->particles[i].pp = p_pose;
  }

  // -------------------------------------------------------------------------------
  // ---- Update debug PF settings
  // -------------------------------------------------------------------------------
  pf->use_landmarks    = params.use_landmarks;
  pf->use_corners      = params.use_corners;
  pf->use_planars      = params.use_planars;
  pf->use_planes       = params.use_planes;
  pf->use_ground_plane = params.use_ground_plane;
  pf->use_icp          = params.use_icp;
  pf->use_gps          = params.use_gps;

  // -------------------------------------------------------------------------------
  // ---- Call particle filter update routine
  // -------------------------------------------------------------------------------
  // - Wait until normal vineslam procedure end
  while (ros::ok())
    if (have_iterated)
      break;

  // - Particle filter update
  pf->update(obsv.landmarks,
             obsv.corners,
             obsv.planars,
             obsv.planes,
             obsv.ground_plane,
             obsv.surf_features,
             obsv.gps_pose,
             previous_map,
             m_grid_map);
//  pf->normalizeWeights();

  // - Get particle with higher weight
  pose  f_pose;
  float w_max = 0;
  for (const auto& particle : pf->particles) {
    std::cout << particle.w << ", ";
    if (particle.w > w_max) {
      w_max  = particle.w;
      f_pose = particle.p;
    }
  }
  std::cout << "\n--\n";

  // -------------------------------------------------------------------------------
  // ---- Publish output data
  // -------------------------------------------------------------------------------
  // - Debug particles
  geometry_msgs::PoseArray ros_poses;
  ros_poses.header.stamp    = ros::Time::now();
  ros_poses.header.frame_id = "odom";
  for (const auto& particle : pf->particles) {
    tf::Quaternion m_q;
    m_q.setRPY(particle.p.roll, particle.p.pitch, particle.p.yaw);
    m_q.normalize();

    geometry_msgs::Pose m_pose;
    m_pose.position.x    = particle.p.x;
    m_pose.position.y    = particle.p.y;
    m_pose.position.z    = particle.p.z;
    m_pose.orientation.x = m_q.x();
    m_pose.orientation.y = m_q.y();
    m_pose.orientation.z = m_q.z();
    m_pose.orientation.w = m_q.w();

    ros_poses.poses.push_back(m_pose);
  }
  debug_pf_particles_pub.publish(ros_poses);

  // - Particles weights
  visualization_msgs::MarkerArray spheres;
  for (const auto& particle : pf->particles) {
    visualization_msgs::Marker m_sphere;
    m_sphere.header.frame_id    = "odom";
    m_sphere.header.stamp       = ros::Time::now();
    m_sphere.ns                 = "sphere_" + std::to_string(particle.id);
    m_sphere.id                 = particle.id;
    m_sphere.type               = visualization_msgs::Marker::SPHERE;
    m_sphere.action             = visualization_msgs::Marker::ADD;
    m_sphere.color.a            = 1;
    m_sphere.color.r            = 0;
    m_sphere.color.b            = 1;
    m_sphere.color.g            = 0;
    m_sphere.pose.position.x    = particle.p.x;
    m_sphere.pose.position.y    = particle.p.y;
    m_sphere.pose.position.z    = particle.p.z;
    m_sphere.pose.orientation.x = 0;
    m_sphere.pose.orientation.y = 0;
    m_sphere.pose.orientation.z = 0;
    m_sphere.pose.orientation.w = 1;
    m_sphere.scale.x            = particle.w * (0.1 / w_max);
    m_sphere.scale.y            = particle.w * (0.1 / w_max);
    m_sphere.scale.z            = particle.w * (0.1 / w_max);

    spheres.markers.push_back(m_sphere);
  }
  debug_pf_weights_pub.publish(spheres);

  // - Local maps
  publish3DMap(f_pose, obsv.corners, debug_pf_corners_local_pub);
  publish3DMap(f_pose, obsv.planars, debug_pf_planars_local_pub);
  publish3DMap(f_pose, obsv.planes, debug_pf_planes_local_pub);

  ROS_INFO("Particle filter debugging procedure has ended. Results are being "
           "published!...");
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
}

} // namespace vineslam
