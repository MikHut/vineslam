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

  // Set initialization flags default values
  init         = true;
  init_gps     = true;
  init_odom    = true;
  register_map = true;

  // Load param
  if (!nh.getParam("/slam_node/SLAMNode/config_path", config_path)) {
    ROS_ERROR("/config_path parameter not found. Shutting down...");
    return;
  }

  // Load config file
  auto config = YAML::LoadFile(config_path);
  // Load camera info parameters
  img_width  = config["camera_info"]["img_width"].as<int>();
  img_height = config["camera_info"]["img_height"].as<int>();
  fx         = config["camera_info"]["fx"].as<float>();
  fy         = config["camera_info"]["fy"].as<float>();
  cx         = config["camera_info"]["cx"].as<float>();
  cy         = config["camera_info"]["cy"].as<float>();
  // Load occupancy grid map parameters
  occ_origin.x = config["multilayer_mapping"]["grid_map"]["origin"]["x"].as<float>();
  occ_origin.y = config["multilayer_mapping"]["grid_map"]["origin"]["y"].as<float>();
  occ_resolution =
      config["multilayer_mapping"]["grid_map"]["resolution"].as<float>();
  occ_width     = config["multilayer_mapping"]["grid_map"]["width"].as<float>();
  occ_height    = config["multilayer_mapping"]["grid_map"]["height"].as<float>();
  use_gps       = config["system"]["use_gps"].as<bool>();
  gps_init_lat  = config["system"]["gps_datum"]["lat"].as<float>();
  gps_init_long = config["system"]["gps_datum"]["long"].as<float>();

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
  sync.registerCallback(boost::bind(&SLAMNode::mainCallbackFct, this, _1, _2, _3));

  // Scan subscription
  ros::Subscriber scan_subscriber =
      nh.subscribe("/scan_3D", 1, &SLAMNode::scanListener, this);
  // Odometry subscription
  ros::Subscriber odom_subscriber =
      nh.subscribe("/odom", 1, &SLAMNode::odomListener, this);
  // GPS subscription
  ros::Subscriber gps_subscriber =
      nh.subscribe("/fix", 1, &SLAMNode::gpsListener, this);

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
  pose_publisher  = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/pose", 1);
  gps_publisher   = nh.advertise<geometry_msgs::PoseStamped>("/vineslam/gps", 1);
  path_publisher  = nh.advertise<nav_msgs::Path>("/vineslam/path", 1);
  poses_publisher = nh.advertise<geometry_msgs::PoseArray>("/vineslam/poses", 1);

  // ROS services
  ros::ServiceServer start_reg_srv =
      nh.advertiseService("start_registration", &SLAMNode::startRegistration, this);
  ros::ServiceServer stop_reg_srv =
      nh.advertiseService("stop_registration", &SLAMNode::stopRegistration, this);

  // GNSS varibales
  if (use_gps) {
    datum_autocorrection_stage = 0;
    global_counter             = 0;
  }

  ros::spin();
  ROS_INFO("ROS shutting down...");
}

SLAMNode::~SLAMNode()
{
  // Save map data
  auto config   = YAML::LoadFile(config_path);
  bool save_map = config["multilayer_mapping"]["grid_map"]["save_map"].as<bool>();

  if (save_map) {
    std::cout << "Writing map to file ..." << std::endl;
    MapWriter mw(config_path);
    mw.writeToFile(*grid_map);
  }

  // Save path data
  saveRobotPath(gps_path, robot_path);
}

// --------------------------------------------------------------------------------
// ----- Callbacks and observation functions
// --------------------------------------------------------------------------------

bool SLAMNode::startRegistration(vineslam_ros::start_map_registration::Request&,
                                 vineslam_ros::start_map_registration::Response&)
{
  ROS_INFO("Activating map registration ...\n");
  register_map = true;
  return true;
}

bool SLAMNode::stopRegistration(vineslam_ros::stop_map_registration::Request&,
                                vineslam_ros::stop_map_registration::Response&)
{
  ROS_INFO("Deactivating map registration ...\n");
  register_map = false;
  return true;
}

void SLAMNode::mainCallbackFct(const sensor_msgs::ImageConstPtr& left_image,
                               const sensor_msgs::ImageConstPtr& depth_image,
                               const vision_msgs::Detection2DArrayConstPtr& dets)
{
  // Declaration of the arrays that will constitute the SLAM observations
  std::vector<int>   labels;
  std::vector<float> bearings;
  std::vector<float> depths;

  // -------------------------------------------------------------------------------
  // ---- Extract high-level semantic features
  // -------------------------------------------------------------------------------
  // Loop over all the bounding box detections
  for (const auto& detection : (*dets).detections) {
    // Load a single bounding box detection
    vision_msgs::BoundingBox2D m_bbox = detection.bbox;

    // Calculate the bearing and depth of the detected object
    float depth;
    float bearing;
    computeObsv(*depth_image,
                static_cast<int>(m_bbox.center.x - m_bbox.size_x / 2),
                static_cast<int>(m_bbox.center.y - m_bbox.size_y / 2),
                static_cast<int>(m_bbox.center.x + m_bbox.size_x / 2),
                static_cast<int>(m_bbox.center.y + m_bbox.size_y / 2),
                depth,
                bearing);

    // Check if the calculated depth is valid
    if (depth == -1)
      continue;

    // Insert the measures in the observations arrays
    labels.push_back(detection.results[0].id);
    depths.push_back(depth);
    bearings.push_back(bearing);
  }

  // -------------------------------------------------------------------------------
  // ---- Localization and mapping procedures
  // -------------------------------------------------------------------------------

  // - Data needed to compute the maps
  cv::Mat img =
      cv_bridge::toCvShare(left_image, sensor_msgs::image_encodings::BGR8)->image;
  auto* raw_depths = (float*)(&(*depth_image).data[0]);

  std::vector<ImageFeature> m_imgfeatures;

  if (init && !init_odom && !init_gps && bearings.size() > 1) {
    // Initialize the localizer and get first particles distribution
    localizer->init(pose(0, 0, 0, 0, 0, odom.yaw));
    robot_pose = localizer->getPose();

    // Get static sensor tfs
    tf::TransformListener listener;
    tf::StampedTransform  cam2base;
    try {
      ROS_INFO("Looking for transform from /base_link to /zed_left_camera_frame...");
      listener.waitForTransform("/zed_left_camera_frame",
                                "/base_link",
                                ros::Time::now(),
                                ros::Duration(1.0));
      listener.lookupTransform(
          "/zed_left_camera_frame", "/base_link", ros::Time::now(), cam2base);
      tf::Vector3 t = cam2base.getOrigin();
      tfScalar    roll, pitch, yaw;
      cam2base.getBasis().getRPY(roll, pitch, yaw);

      mapper3D->setCam2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);
      mapper2D->setCamPitch(pitch);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ROS_ERROR("Setting transformation as identity...");
      mapper3D->setCam2Base(0., 0., 0., 0., 0., 0.);
      mapper2D->setCamPitch(0.);

      return;
    }
    tf::StampedTransform vel2base;
    try {
      ROS_INFO("Looking for transform from /base_link to /velodyne...");
      listener.waitForTransform(
          "/velodyne", "/base_link", ros::Time::now(), ros::Duration(1.0));
      listener.lookupTransform(
          "/velodyne", "/base_link", ros::Time::now(), vel2base);
      tf::Vector3 t = vel2base.getOrigin();
      tfScalar    roll, pitch, yaw;
      vel2base.getBasis().getRPY(roll, pitch, yaw);

      mapper3D->setVel2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ROS_ERROR("Setting transformation as identity...");
      mapper3D->setVel2Base(0., 0., 0., 0., 0., 0.);

      return;
    }

    ROS_INFO("Got the transforms!");

    if (register_map) {
      // ---- Initialize the multi-layer map
      // - 2D semantic feature map
      mapper2D->init(robot_pose, bearings, depths, labels, *grid_map);
      // - 3D PCL corner map estimation
      std::vector<Corner> m_corners;
      Plane               m_ground_plane;
      mapper3D->localPCLMap(raw_depths, m_corners, m_ground_plane);
      mapper3D->globalCornerMap(m_corners, robot_pose, *grid_map);
      // - 3D image feature map estimation
      std::vector<ImageFeature> m_surf_features;
      mapper3D->localSurfMap(img, raw_depths, m_surf_features);
      mapper3D->globalSurfMap(m_surf_features, robot_pose, *grid_map);
    }

    init = false;
  } else if (!init && !init_odom && !init_gps) {

    // --------- Build local maps to use in the localization
    // - Compute 2D local map of semantic features on robot's referential frame
    std::vector<SemanticFeature> m_landmarks;
    mapper2D->localMap(bearings, depths, m_landmarks);
    // - Compute 3D PCL corners and ground plane on robot's referential frame
    std::vector<Corner> m_corners;
    Plane               m_ground_plane;
    mapper3D->localPCLMap(raw_depths, m_corners, m_ground_plane);
    // - Compute 3D image features on robot's referential frame
    std::vector<ImageFeature> m_surf_features;
    mapper3D->localSurfMap(img, raw_depths, m_surf_features);

    // ------- Build observation structure to use in the localization
    Observation obsv;
    obsv.landmarks    = m_landmarks;
    obsv.corners      = m_corners;
    obsv.ground_plane = m_ground_plane;
    if (has_converged && use_gps)
      obsv.gps_pose = gps_pose;
    else
      obsv.gps_pose = pose(0., 0., 0., 0., 0., 0.);
    obsv.surf_features = m_surf_features;

    // ------- LOCALIZATION PROCEDURE ---------- //
    localizer->process(odom, obsv, *grid_map);
    robot_pose = localizer->getPose();

    if (register_map) {
      // ------- MULTI-LAYER MAPPING REGISTRATION ------------ //
      // - 2D high-level semantic map estimation
      mapper2D->process(robot_pose, m_landmarks, labels, *grid_map);
      // - 3D PCL corner map estimation
      mapper3D->globalCornerMap(m_corners, robot_pose, *grid_map);
      // - 3D image feature map estimation
      mapper3D->globalSurfMap(m_surf_features, robot_pose, *grid_map);
      // ---------------------------------------- //
    }

    // Save poses to paths
    std::array<float, 9> robot_R{};
    robot_pose.toRotMatrix(robot_R);
    TF robot_tf(robot_R,
                std::array<float, 3>{robot_pose.x, robot_pose.y, robot_pose.z});
    robot_path.push_back(robot_tf);
    std::array<float, 9> gps_R{};
    robot_pose.toRotMatrix(gps_R);
    TF gps_tf(robot_R, std::array<float, 3>{gps_pose.x, gps_pose.y, gps_pose.z});
    gps_path.push_back(gps_tf);

    // Convert robot pose to tf::Transform corresponding
    // to the camera to map transformation
    tf::Quaternion q;
    q.setRPY(robot_pose.roll, robot_pose.pitch, robot_pose.yaw);
    q.normalize();
    tf::Transform base2map;
    base2map.setRotation(q);
    base2map.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, robot_pose.z));

    // Convert vineslam pose to ROS pose and publish it
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header             = depth_image->header;
    pose_stamped.header.frame_id    = "map";
    pose_stamped.pose.position.x    = robot_pose.x;
    pose_stamped.pose.position.y    = robot_pose.y;
    pose_stamped.pose.position.z    = robot_pose.z;
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    pose_publisher.publish(pose_stamped);

    // Push back the current pose to the path container and publish it
    path.push_back(pose_stamped);
    nav_msgs::Path ros_path;
    ros_path.header          = depth_image->header;
    ros_path.header.frame_id = "map";
    ros_path.poses           = path;
    path_publisher.publish(ros_path);

    // Publish cam-to-map tf::Transform
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(
        base2map, pose_stamped.header.stamp, "map", "base_link"));

    // ---------- Publish Multi-layer map ------------- //
    // Publish the grid map
    publishGridMap(depth_image->header);
    // Publish the 2D map
    publish2DMap(depth_image->header, robot_pose, bearings, depths);
    // Publish 3D maps
    publish3DMap();
    // ------------------------------------------------ //

#ifdef DEBUG
    // Publish all poses for DEBUG
    // ----------------------------------------------------------------------------
    std::vector<pose> poses;
    (*localizer).getParticles(poses);
    geometry_msgs::PoseArray ros_poses;
    ros_poses.header          = depth_image->header;
    ros_poses.header.frame_id = "map";
    for (const auto& pose : poses) {
      tf::Quaternion m_q;
      m_q.setRPY(pose.roll, pose.pitch, pose.yaw);
      m_q.normalize();

      geometry_msgs::Pose m_pose;
      m_pose.position.x    = pose.x;
      m_pose.position.y    = pose.y;
      m_pose.position.z    = pose.z;
      m_pose.orientation.x = m_q.x();
      m_pose.orientation.y = m_q.y();
      m_pose.orientation.z = m_q.z();
      m_pose.orientation.w = m_q.w();

      ros_poses.poses.push_back(m_pose);
    }
    poses_publisher.publish(ros_poses);

    // - Compute rotation matrix from ground plane normal vector and aplly it to the
    // plane
    std::array<float, 9> R{};
    vector3D             m_normal = obsv.ground_plane.normal;
    float norm = std::sqrt(m_normal.x * m_normal.x + m_normal.y * m_normal.y);
    R[0]       = +m_normal.y / norm;
    R[1]       = -m_normal.x / norm;
    R[2]       = 0.;
    R[3]       = (m_normal.x * m_normal.z) / norm;
    R[4]       = (m_normal.y * m_normal.z) / norm;
    R[5]       = -norm;
    R[6]       = m_normal.x;
    R[7]       = m_normal.y;
    R[8]       = m_normal.z;

    std::array<float, 3> trans            = {0., 0., 0.};
    pose                 pose_from_ground = pose(R, trans);
    pose_from_ground.yaw                  = 0.;
    R                                     = {};
    pose_from_ground.toRotMatrix(R);

    Plane m_plane;
    for (const auto& pt : obsv.ground_plane.points) {
      point m_pt;
      m_pt.x = pt.x * R[0] + pt.y * R[1] + pt.z * R[2];
      m_pt.y = pt.x * R[3] + pt.y * R[4] + pt.z * R[5];
      m_pt.z = pt.x * R[6] + pt.y * R[7] + pt.z * R[8];

      m_plane.points.push_back(pt);
    }
    publish3DMap(m_plane, map3D_planes_publisher);
#endif
  }
}

void SLAMNode::scanListener(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_pcl(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *velodyne_pcl);
  // Remove Nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*velodyne_pcl, *velodyne_pcl, indices);

  scan_pts.clear();
  for (const auto& pt : *velodyne_pcl) {
    point m_pt(pt.x, pt.y, pt.z);
    scan_pts.push_back(m_pt);
  }
}

void SLAMNode::odomListener(const nav_msgs::OdometryConstPtr& msg)
{
  // Convert odometry msg to pose msg
  tf::Pose            pose;
  geometry_msgs::Pose odom_pose = (*msg).pose.pose;
  tf::poseMsgToTF(odom_pose, pose);

  // Check if yaw is NaN
  float yaw = static_cast<float>(tf::getYaw(pose.getRotation()));
  if (!std::isfinite(yaw))
    yaw = 0;

  // If it is the first iteration - initialize the Pose
  // relative to the previous frame
  if (init_odom) {
    p_odom.x   = (*msg).pose.pose.position.x;
    p_odom.y   = (*msg).pose.pose.position.y;
    p_odom.yaw = yaw;
    odom       = vineslam::pose(0., 0., 0., 0., 0., yaw);
    init_odom  = false;
    return;
  }

  // Integrate odometry pose to convert to the map frame
  odom.x += static_cast<float>(msg->pose.pose.position.x) - p_odom.x;
  odom.y += static_cast<float>(msg->pose.pose.position.y) - p_odom.y;
  odom.z     = 0;
  odom.roll  = 0;
  odom.pitch = 0;
  odom.yaw += (yaw - p_odom.yaw);

  // Save current odometry pose to use in the next iteration
  p_odom.x   = msg->pose.pose.position.x;
  p_odom.y   = msg->pose.pose.position.y;
  p_odom.yaw = yaw;
}

void SLAMNode::gpsListener(const sensor_msgs::NavSatFixConstPtr& msg)
{
  if (init_gps) {
    has_converged = false;

    // Set initial datum
    agrob_map_transform::SetDatum srv;
    srv.request.geo_pose.position.latitude  = gps_init_lat;
    srv.request.geo_pose.position.longitude = gps_init_long;
    srv.request.geo_pose.position.altitude  = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(quat, srv.request.geo_pose.orientation);

    set_datum.call(srv);

    init_gps = false;
  }

  agrob_map_transform::GetPose srv;

  // GNSS - odom service call
  srv.request.geo_pose.latitude  = msg->latitude;
  srv.request.geo_pose.longitude = msg->longitude;

  if (polar2pose.call(srv)) {
    pose gps_odom;
    gps_odom.x = srv.response.local_pose.pose.pose.position.x;
    gps_odom.y = srv.response.local_pose.pose.pose.position.y;

    has_converged = getGNSSHeading(gps_odom, msg->header);

    // Compute the gnss to map transform
    tf::Quaternion heading_quat;
    heading_quat.setRPY(0., 0., heading);
    heading_quat.normalize();
    tf::Transform ned2map(heading_quat, tf::Vector3(0., 0., 0.));

    // Publish gnss to map tf::Transform
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(ned2map, msg->header.stamp, "enu", "map"));

    // Publish gnss pose in the enu reference frame
    geometry_msgs::PoseStamped gnss_pose;
    gnss_pose.pose.position.x    = gps_odom.x;
    gnss_pose.pose.position.y    = gps_odom.y;
    gnss_pose.pose.position.z    = gps_odom.z;
    gnss_pose.pose.orientation.x = 0.;
    gnss_pose.pose.orientation.y = 0.;
    gnss_pose.pose.orientation.z = 0.;
    gnss_pose.pose.orientation.w = 1.;
    gnss_pose.header             = msg->header;
    gnss_pose.header.frame_id    = "enu";
    gps_publisher.publish(gnss_pose);

    // Transform locally the gps pose from enu to map to use in localization
    tf::Matrix3x3 Rot = ned2map.getBasis().inverse();

    gps_pose.x = static_cast<float>(Rot[0].getX()) * gps_odom.x +
                 static_cast<float>(Rot[0].getY()) * gps_odom.y +
                 static_cast<float>(Rot[0].getZ()) * gps_odom.z;
    gps_pose.y = static_cast<float>(Rot[1].getX()) * gps_odom.x +
                 static_cast<float>(Rot[1].getY()) * gps_odom.y +
                 static_cast<float>(Rot[1].getZ()) * gps_odom.z;
    gps_pose.z     = 0.;
    gps_pose.roll  = 0.;
    gps_pose.pitch = 0.;
    gps_pose.yaw   = 0.;

  } else {
    ROS_ERROR("Failed to call service Polar2Pose\n");
    return;
  }
}

bool SLAMNode::getGNSSHeading(const pose& gps_odom, const std_msgs::Header& header)
{
  float weight_max = 0.;
  if (datum_autocorrection_stage == 0) {
    ROS_DEBUG("Initialization of AGROB DATUM");
    datum_autocorrection_stage++;
  } else {

    float x, y;
    x = robot_pose.x;
    y = robot_pose.y;

    float distance   = std::sqrt((gps_odom.x - x) * (gps_odom.x - x) +
                               (gps_odom.y - y) * (gps_odom.y - y));
    float center_map = std::sqrt(gps_odom.x * gps_odom.x + gps_odom.y * gps_odom.y);

    if (datum_autocorrection_stage == 1) {
      if (center_map < 2.0) {
        if (distance < 5.0) {
          datum_autocorrection_stage = 2;

        } else {
          ROS_ERROR("Datum localization is bad. Error on heading location.");
          datum_autocorrection_stage = -1;
        }

      } else {
        ROS_ERROR("Error on heading location.");
        datum_autocorrection_stage = -1;
      }
    } else if (datum_autocorrection_stage == 2) {
      ROS_DEBUG("Initializing datum filter.");
      for (int i = 0; i < 360; i++) {
        datum_orientation[i][0] = static_cast<float>(i);
        datum_orientation[i][1] = 1.0;
      }
      datum_autocorrection_stage = 3;
    } else if (datum_autocorrection_stage == 3) {
      global_counter++;

      float dist_temp_max = 0.0;
      for (auto& i : datum_orientation) {
        float xtemp, ytemp, dist_temp;
        xtemp =
            std::cos(i[0] * DEGREE_TO_RAD) * x - std::sin(i[0] * DEGREE_TO_RAD) * y;
        ytemp =
            std::sin(i[0] * DEGREE_TO_RAD) * x + std::cos(i[0] * DEGREE_TO_RAD) * y;
        dist_temp = std::sqrt((gps_odom.x - xtemp) * (gps_odom.x - xtemp) +
                              (gps_odom.y - ytemp) * (gps_odom.y - ytemp));
        i[2]      = dist_temp;
        if (dist_temp_max < dist_temp)
          dist_temp_max = dist_temp;
      }

      int indexT = 0, index = 0;
      for (auto& i : datum_orientation) {

        i[1] = (i[1] * static_cast<float>(global_counter) +
                static_cast<float>(1. - i[2] / dist_temp_max) * center_map) /
               static_cast<float>(global_counter + center_map);

        if (weight_max < i[1]) {
          weight_max = i[1];
          indexT     = index;
        }

        index++;
      }

      if (weight_max > 0.) {
        heading = static_cast<float>(indexT) * DEGREE_TO_RAD;
        ROS_DEBUG("Solution = %d.", indexT);
      } else
        ROS_INFO("Did not find any solution for datum heading.");

    } else
      ROS_ERROR("Datum localization is bad. Error on heading location.");
  }

  std::cout << weight_max << std::endl;
  return weight_max > 0.6;
}

void SLAMNode::computeObsv(const sensor_msgs::Image& depth_img,
                           const int&                xmin,
                           const int&                ymin,
                           const int&                xmax,
                           const int&                ymax,
                           float&                    depth,
                           float&                    bearing) const
{
  // Declare array with all the disparities computed
  auto* depths = (float*)(&(depth_img).data[0]);

  // Set minimum and maximum depth values to consider
  float range_min = 0.01;
  float range_max = 10.0;

  std::map<float, float> dtheta;
  for (int i = xmin; i < xmax; i++) {
    for (int j = ymin; j < ymax; j++) {
      int idx = i + depth_img.width * j;

      // Fill the depth array with the values of interest
      if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
          depths[idx] < range_max) {
        float x         = depths[idx];
        float y         = -(static_cast<float>(i) - cx) * (x / fx);
        float m_depth   = static_cast<float>(sqrt(pow(x, 2) + pow(y, 2)));
        dtheta[m_depth] = atan2(y, x);
      }
    }
  }

  // compute minimum of all observations
  size_t n_depths = dtheta.size();
  if (n_depths > 0) {
    depth   = dtheta.begin()->first;
    bearing = dtheta.begin()->second;
  } else {
    depth   = -1;
    bearing = -1;
  }
}

// --------------------------------------------------------------------------------
// ----- Visualization
// --------------------------------------------------------------------------------

void SLAMNode::publishGridMap(const std_msgs::Header& header)
{
  // Define ROS occupancy grid map
  nav_msgs::OccupancyGrid occ_map;
  occ_map.header          = header;
  occ_map.header.frame_id = "map";

  // Set the map metadata
  nav_msgs::MapMetaData metadata;
  metadata.origin.position.x    = occ_origin.x;
  metadata.origin.position.y    = occ_origin.y;
  metadata.origin.position.z    = occ_origin.z;
  metadata.origin.orientation.x = 0.;
  metadata.origin.orientation.y = 0.;
  metadata.origin.orientation.z = 0.;
  metadata.origin.orientation.w = 1.;
  metadata.resolution           = occ_resolution;
  metadata.width                = occ_width / occ_resolution;
  metadata.height               = occ_height / occ_resolution;
  occ_map.info                  = metadata;

  // Fill the occupancy grid map
  occ_map.data.resize(metadata.width * metadata.height);
  // - compute x and y bounds
  int xmin = static_cast<int>(occ_origin.x / occ_resolution);
  int xmax = static_cast<int>((float)xmin + occ_width / occ_resolution - 1);
  int ymin = static_cast<int>(occ_origin.y / occ_resolution);
  int ymax = static_cast<int>((float)ymin + occ_height / occ_resolution - 1);
  for (int i = xmin; i < xmax; i++) {
    for (int j = ymin; j < ymax; j++) {
      int8_t number_objs = (*grid_map)(i, j).landmarks.size() +
                           (*grid_map)(i, j).surf_features.size() +
                           (*grid_map)(i, j).corner_features.size();

      int m_i = i - static_cast<int>(occ_origin.x / occ_resolution);
      int m_j = j - static_cast<int>(occ_origin.y / occ_resolution);
      int idx = m_i + m_j * static_cast<int>((occ_width / occ_resolution));

      occ_map.data[idx] = number_objs * 10;
    }
  }

  // Publish the map
  mapOCC_publisher.publish(occ_map);
}

void SLAMNode::publish2DMap(const std_msgs::Header&   header,
                            const pose&               pose,
                            const std::vector<float>& bearings,
                            const std::vector<float>& depths)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker      marker;
  visualization_msgs::MarkerArray ellipse_array;
  visualization_msgs::Marker      ellipse;

  // Define marker layout
  marker.ns                 = "/markers";
  marker.type               = visualization_msgs::Marker::CYLINDER;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.scale.x            = 0.1;
  marker.scale.y            = 0.1;
  marker.scale.z            = 0.3;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.r            = 0.0f;
  marker.color.g            = 0.0f;
  marker.color.b            = 1.0f;
  marker.color.a            = 1.0;
  marker.lifetime           = ros::Duration();

  // Define marker layout
  ellipse.ns                 = "/ellipses";
  ellipse.type               = visualization_msgs::Marker::CYLINDER;
  ellipse.action             = visualization_msgs::Marker::ADD;
  ellipse.scale.z            = 0.01f;
  ellipse.pose.orientation.x = 0.0f;
  ellipse.pose.orientation.y = 0.0f;
  ellipse.color.r            = 0.0f;
  ellipse.color.g            = 1.0f;
  ellipse.color.b            = 0.0f;
  ellipse.color.a            = 1.0f;
  ellipse.lifetime           = ros::Duration();

  // Publish markers
  int id = 1;
  for (auto& it : (*grid_map)) {
    for (const auto& m_sfeature : it.landmarks) {
      // Draw sfeature mean
      marker.id              = id;
      marker.header          = header;
      marker.header.frame_id = "map";
      marker.pose.position.x = m_sfeature.second.pos.x;
      marker.pose.position.y = m_sfeature.second.pos.y;
      marker.pose.position.z = m_sfeature.second.pos.z;

      marker_array.markers.push_back(marker);

      // Draw sfeature standard deviation
      tf2::Quaternion q;
      q.setRPY(0, 0, m_sfeature.second.gauss.theta);

      ellipse.id                 = id;
      ellipse.header             = header;
      ellipse.header.frame_id    = "map";
      ellipse.pose.position.x    = m_sfeature.second.pos.x;
      ellipse.pose.position.y    = m_sfeature.second.pos.y;
      ellipse.pose.position.z    = m_sfeature.second.pos.z;
      ellipse.scale.x            = 3 * m_sfeature.second.gauss.stdev.x;
      ellipse.scale.y            = 3 * m_sfeature.second.gauss.stdev.y;
      ellipse.pose.orientation.x = q.x();
      ellipse.pose.orientation.y = q.y();
      ellipse.pose.orientation.z = q.z();
      ellipse.pose.orientation.w = q.w();

      ellipse_array.markers.push_back(ellipse);

      id++;
    }
  }

  // Draw ellipse that characterizes particles distribution
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.dist.theta);

  ellipse.id                 = id;
  ellipse.header             = header;
  ellipse.header.frame_id    = "map";
  ellipse.pose.position.x    = pose.x;
  ellipse.pose.position.y    = pose.y;
  ellipse.pose.position.z    = pose.z;
  ellipse.scale.x            = 3 * pose.dist.stdev.x;
  ellipse.scale.y            = 3 * pose.dist.stdev.y;
  ellipse.pose.orientation.x = q.x();
  ellipse.pose.orientation.y = q.y();
  ellipse.pose.orientation.z = q.z();
  ellipse.pose.orientation.w = q.w();
  ellipse.color.r            = 0.0f;
  ellipse.color.g            = 0.0f;
  ellipse.color.b            = 1.0f;
  ellipse.color.a            = 1.0f;
  ellipse_array.markers.push_back(ellipse);

  map2D_publisher.publish(marker_array);
  map2D_publisher.publish(ellipse_array);
}

void SLAMNode::publish3DMap()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (const auto& it : *grid_map) {
    for (const auto& feature : it.surf_features) {
      pcl::PointXYZRGB m_pt(feature.r, feature.g, feature.b);
      m_pt.x = feature.pos.x;
      m_pt.y = feature.pos.y;
      m_pt.z = feature.pos.z;

      feature_cloud->points.push_back(m_pt);
    }

    for (const auto& corner : it.corner_features) {
      pcl::PointXYZI m_pt(corner.which_plane);
      m_pt.x = corner.pos.x;
      m_pt.y = corner.pos.y;
      m_pt.z = corner.pos.z;

      corner_cloud->points.push_back(m_pt);
    }
  }

  feature_cloud->header.frame_id = "map";
  corner_cloud->header.frame_id  = "map";
  map3D_features_publisher.publish(feature_cloud);
  map3D_corners_publisher.publish(corner_cloud);
}

void SLAMNode::publish3DMap(const Plane& plane, const ros::Publisher& pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointXYZI m_pt(0);

  for (const auto& pt : plane.points) {
    m_pt.x = pt.x;
    m_pt.y = pt.y;
    m_pt.z = pt.z;

    cloud_out->points.push_back(m_pt);
  }

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

void SLAMNode::publish3DMap(const std::vector<Corner>& corners,
                            const ros::Publisher&      pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointXYZI m_pt(0);

  for (const auto& corner : corners) {
    m_pt.x = corner.pos.x;
    m_pt.y = corner.pos.y;
    m_pt.z = corner.pos.z;

    cloud_out->points.push_back(m_pt);
  }

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

} // namespace vineslam
