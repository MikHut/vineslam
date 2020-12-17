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
  debug_pf_ = new PF(params_, Pose(0, 0, 0, 0, 0, 0));

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
  map2D_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map2D", 1);
  map3D_features_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/vineslam/map3D/SURF", 1);
  map3D_corners_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners", 1);
  map3D_planars_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars", 1);
  map3D_planes_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map3D/planes", 1);
  corners_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/corners_local", 1);
  planars_local_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/map3D/planars_local", 1);
  planes_local_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/map3D/planes_local", 1);
  pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  gps_publisher_ = nh.advertise<nav_msgs::Path>("/vineslam/gps", 1);
  path_publisher_ = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher_ = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);
  // Debug publishers
  debug_pf_particles_pub_ = nh.advertise<geometry_msgs::PoseArray>("/vineslam/debug/poses", 1);
  debug_pf_weights_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/vineslam/debug/weights", 1);
  debug_pf_corners_local_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/debug/map3D/corners_local", 1);
  debug_pf_planars_local_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/debug/map3D/planars_local", 1);
  debug_pf_planes_local_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/debug/map3D/planes_local", 1);
  debug_pf_ground_local_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/vineslam/debug/map3D/ground_local", 1);

  // ROS services
  ros::ServiceServer start_reg_srv =
      nh.advertiseService("start_registration", &VineSLAM_ros::startRegistration, dynamic_cast<VineSLAM_ros*>(this));
  ros::ServiceServer stop_reg_srv =
      nh.advertiseService("stop_registration", &VineSLAM_ros::stopRegistration, dynamic_cast<VineSLAM_ros*>(this));
  ros::ServiceServer stop_hed_srv = nh.advertiseService(
      "stop_gps_heading_estimation", &VineSLAM_ros::stopHeadingEstimation, dynamic_cast<VineSLAM_ros*>(this));
  ros::ServiceServer change_node_state_srv =
      nh.advertiseService("change_replay_node_state", &ReplayNode::changeNodeState, this);
  ros::ServiceServer change_node_features_srv =
      nh.advertiseService("change_replay_node_features", &ReplayNode::changeNodeFeatures, this);
  ros::ServiceServer debug_pf_srv = nh.advertiseService("debug_pf", &ReplayNode::debugPF, this);

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
  ros::Publisher depth_img_pub = nh.advertise<sensor_msgs::Image>(params_.depth_img_topic_, 1);
  ros::Publisher left_img_pub = nh.advertise<sensor_msgs::CompressedImage>(params_.rgb_img_topic_, 1);
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(params_.pcl_topic_, 1);

  rosgraph_msgs::Clock clock_ptr;
  tf2_msgs::TFMessageConstPtr tf_ptr;
  nav_msgs::OdometryConstPtr rs_odom_ptr;
  nav_msgs::OdometryConstPtr odom_ptr;
  sensor_msgs::NavSatFixConstPtr fix_ptr;
  sensor_msgs::ImageConstPtr depth_img_ptr;
  cv::Mat left_img;
  sensor_msgs::PointCloud2ConstPtr pcl_ptr;

  ROS_INFO("Reading ROSBAG topics...");

  bool tf_bool = false, odom_bool = false, fix_bool = false, depth_bool = false, left_bool = false, pcl_bool = false;
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
        fix_bool = true;
        fix_pub.publish(*fix_ptr);
      }
    }
    else if (topic == params_.depth_img_topic_)
    {
      depth_img_ptr = m.instantiate<sensor_msgs::Image>();

      if (depth_img_ptr != nullptr)
      {
        depth_bool = true;
        depth_img_pub.publish(*depth_img_ptr);
      }
    }
    else if (topic == params_.rgb_img_topic_)
    {
      sensor_msgs::CompressedImageConstPtr left_img_comp_ptr = m.instantiate<sensor_msgs::CompressedImage>();

      if (left_img_comp_ptr != nullptr)
      {
        left_bool = true;
        left_img_pub.publish(*left_img_comp_ptr);

        // Decompress image
        cv::Mat img_data(1, left_img_comp_ptr->data.size(), CV_8UC3);
        img_data.data = const_cast<uchar*>(&left_img_comp_ptr->data[0]);
        cv::InputArray data(img_data);
        left_img = cv::imdecode(data, 1);
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

    nmessages_ = tf_bool + odom_bool + fix_bool + depth_bool + left_bool + pcl_bool;

    if (nmessages_ == 6)
    {
      // Save data to use on debug procedure
      if (!init_flag_)
      {
        debug_particles_vec_.clear();
        localizer_->getParticles(debug_particles_vec_);
        debug_grid_map_ = grid_map_;
      }

      _imageListener(left_img, depth_img_ptr);
      scanListener(pcl_ptr);
      odomListener(odom_ptr);
      // TODO (Andre Aguiar): GPS and landmarks should be supported in the future
      // landmarkListener(dets);
      // gpsListener(fix_ptr);

      // Call VineSLAM loop
      loopOnce();

      tf_bool = false;
      odom_bool = false;
      fix_bool = false;
      depth_bool = false;
      left_bool = false;
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

bool ReplayNode::changeNodeFeatures(vineslam_ros::change_replay_node_features::Request& request,
                                    vineslam_ros::change_replay_node_features::Response& response)
{
  params_.use_semantic_features_ = request.use_semantic_features.data;
  params_.use_lidar_features_ = request.use_lidar_features.data;
  params_.use_image_features_ = request.use_image_features.data;
  params_.use_gps_ = request.use_gps.data;

  localizer_->changeObservationsToUse(params_.use_semantic_features_, params_.use_lidar_features_,
                                      params_.use_image_features_, params_.use_gps_);

  return true;
}

bool ReplayNode::debugPF(vineslam_ros::debug_particle_filter::Request& request,
                         vineslam_ros::debug_particle_filter::Response& response)
{
  ROS_INFO("Particle filter debugging procedure has started ...");

  // -------------------------------------------------------------------------------
  // ---- Create set of particles from the limits established by the user
  // -------------------------------------------------------------------------------
  float w = 1 / static_cast<float>(debug_particles_vec_.size());
  for (size_t i = 0; i < debug_particles_vec_.size(); i++)
  {
    Pose p_pose = debug_particles_vec_[i].p_;
    Pose m_pose = debug_particles_vec_[i].p_ +
                  Pose(sampleGaussian(request.x_std), sampleGaussian(request.y_std), sampleGaussian(request.z_std),
                       sampleGaussian(request.R_std * DEGREE_TO_RAD), sampleGaussian(request.P_std * DEGREE_TO_RAD),
                       sampleGaussian(request.Y_std * DEGREE_TO_RAD));

    debug_pf_->particles_[i] = Particle(i, m_pose, w);
    debug_pf_->particles_[i].pp_ = p_pose;
  }

  // -------------------------------------------------------------------------------
  // ---- Update debug PF settings
  // -------------------------------------------------------------------------------
  debug_pf_->use_semantic_features_ = params_.use_semantic_features_;
  debug_pf_->use_lidar_features_ = params_.use_lidar_features_;
  debug_pf_->use_image_features_ = params_.use_image_features_;
  debug_pf_->use_gps_ = params_.use_gps_;

  // -------------------------------------------------------------------------------
  // ---- Call particle filter update routine
  // -------------------------------------------------------------------------------
  // - Wait until normal vineslam procedure ends
  while (ros::ok())
    if (have_iterated_)
      break;

  // - Particle filter update
  debug_pf_->update(obsv_.landmarks, obsv_.corners, obsv_.planars, obsv_.planes, obsv_.ground_plane,
                    obsv_.surf_features, obsv_.gps_pose, debug_grid_map_);

  // - Get particle with higher weight
  Pose f_pose;
  float w_max = 0;
  for (const auto& particle : debug_pf_->particles_)
  {
    std::cout << particle.w_ << ", ";
    if (particle.w_ > w_max)
    {
      w_max = particle.w_;
      f_pose = particle.p_;
    }
  }
  std::cout << "\n--\n";

  // -------------------------------------------------------------------------------
  // ---- Publish output data
  // -------------------------------------------------------------------------------
  // - Debug particles
  geometry_msgs::PoseArray ros_poses;
  ros_poses.header.stamp = ros::Time::now();
  ros_poses.header.frame_id = "map";
  for (const auto& particle : debug_pf_->particles_)
  {
    tf::Quaternion m_q;
    m_q.setRPY(particle.p_.R_, particle.p_.P_, particle.p_.Y_);
    m_q.normalize();

    geometry_msgs::Pose m_pose;
    m_pose.position.x = particle.p_.x_;
    m_pose.position.y = particle.p_.y_;
    m_pose.position.z = particle.p_.z_;
    m_pose.orientation.x = m_q.x();
    m_pose.orientation.y = m_q.y();
    m_pose.orientation.z = m_q.z();
    m_pose.orientation.w = m_q.w();

    ros_poses.poses.push_back(m_pose);
  }
  debug_pf_particles_pub_.publish(ros_poses);

  // - Particles weights
  visualization_msgs::MarkerArray spheres;
  for (const auto& particle : debug_pf_->particles_)
  {
    visualization_msgs::Marker m_sphere;
    m_sphere.header.frame_id = "map";
    m_sphere.header.stamp = ros::Time::now();
    m_sphere.ns = "sphere_" + std::to_string(particle.id_);
    m_sphere.id = particle.id_;
    m_sphere.type = visualization_msgs::Marker::SPHERE;
    m_sphere.action = visualization_msgs::Marker::ADD;
    m_sphere.color.a = 1;
    m_sphere.color.r = 0;
    m_sphere.color.b = 1;
    m_sphere.color.g = 0;
    m_sphere.pose.position.x = particle.p_.x_;
    m_sphere.pose.position.y = particle.p_.y_;
    m_sphere.pose.position.z = particle.p_.z_;
    m_sphere.pose.orientation.x = 0;
    m_sphere.pose.orientation.y = 0;
    m_sphere.pose.orientation.z = 0;
    m_sphere.pose.orientation.w = 1;
    m_sphere.scale.x = particle.w_ * (0.1 / w_max);
    m_sphere.scale.y = particle.w_ * (0.1 / w_max);
    m_sphere.scale.z = particle.w_ * (0.1 / w_max);

    spheres.markers.push_back(m_sphere);
  }
  debug_pf_weights_pub_.publish(spheres);

  // - Local maps
  publish3DMap(f_pose, obsv_.corners, debug_pf_corners_local_pub_);
  publish3DMap(f_pose, obsv_.planars, debug_pf_planars_local_pub_);
  publish3DMap(f_pose, obsv_.planes, debug_pf_planes_local_pub_);
  //  publish3DMap(f_pose, { obsv_.ground_plane }, debug_pf_ground_local_pub_);

  // Convert robot pose to tf::Transform corresponding
  tf::Quaternion q;
  q.setRPY(f_pose.R_, f_pose.P_, f_pose.Y_);
  q.normalize();
  tf::Transform debug2map;
  debug2map.setRotation(q);
  debug2map.setOrigin(tf::Vector3(f_pose.x_, f_pose.y_, f_pose.z_));
  // Publish tf::Trasforms
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(debug2map, ros::Time::now(), "map", "debug"));

  ROS_INFO("Particle filter debugging procedure has ended. Results are being "
           "published!...");
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

ReplayNode::~ReplayNode()
{
}

}  // namespace vineslam
