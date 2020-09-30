#include "../include/vineslam_ros.hpp"

namespace vineslam
{

// --------------------------------------------------------------------------------
// ----- Callbacks and observation functions
// --------------------------------------------------------------------------------

bool VineSLAM_ros::startRegistration(vineslam_ros::start_map_registration::Request&,
                                     vineslam_ros::start_map_registration::Response&)
{
  ROS_INFO("Activating map registration ...\n");
  register_map = true;
  return true;
}

bool VineSLAM_ros::stopRegistration(vineslam_ros::stop_map_registration::Request&,
                                    vineslam_ros::stop_map_registration::Response&)
{
  ROS_INFO("Deactivating map registration ...\n");
  register_map = false;
  return true;
}

bool VineSLAM_ros::stopHeadingEstimation(
    vineslam_ros::stop_gps_heading_estimation::Request&,
    vineslam_ros::stop_gps_heading_estimation::Response&)
{
  ROS_INFO("Deactivating gps heading estimation ...\n");
  estimate_heading = false;
  return true;
}

void VineSLAM_ros::mainCallbackFct(const sensor_msgs::ImageConstPtr& left_image,
                                   const sensor_msgs::ImageConstPtr& depth_image,
                                   const vision_msgs::Detection2DArrayConstPtr& dets)
{
  cv::Mat img =
      cv_bridge::toCvShare(left_image, sensor_msgs::image_encodings::BGR8)->image;

  mainFct(img, depth_image, dets);
}

void VineSLAM_ros::mainFct(const cv::Mat&                               left_image,
                           const sensor_msgs::ImageConstPtr&            depth_image,
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
  if (dets != nullptr) {
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
  }

  // -------------------------------------------------------------------------------
  // ---- Localization and mapping procedures
  // -------------------------------------------------------------------------------

  // - Data needed to compute the maps
  auto* raw_depths = (float*)(&(*depth_image).data[0]);

  std::vector<ImageFeature> m_imgfeatures;

  if (init && !init_odom && (!init_gps || !params.use_gps) &&
      (bearings.size() > 1 || !params.use_landmarks)) {
    // Initialize the localizer and get first particles distribution
    localizer->init(pose(0, 0, 0, 0, 0, odom.yaw));
    robot_pose = localizer->getPose();

    if (register_map) {
      // ---- Initialize the multi-layer map
      // - 2D semantic feature map
      mapper2D->init(robot_pose, bearings, depths, labels, *grid_map);
      // - 3D PCL corner map estimation
      std::vector<Corner>  m_corners;
      std::vector<Cluster> m_clusters;
      std::vector<Line>    m_vegetation_lines;
      Plane                m_ground_plane;
      mapper3D->localPCLMap(
          scan_pts, m_corners, m_clusters, m_vegetation_lines, m_ground_plane);
      mapper3D->globalCornerMap(robot_pose, m_corners, *grid_map);
      // - 3D image feature map estimation
      std::vector<ImageFeature> m_surf_features;
      mapper3D->localSurfMap(left_image, raw_depths, m_surf_features);
      mapper3D->globalSurfMap(m_surf_features, robot_pose, *grid_map);
    }

    // Convert vineslam pose to ROS pose and publish it
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp       = ros::Time::now();
    pose_stamped.header.frame_id    = "map";
    pose_stamped.pose.position.x    = 0;
    pose_stamped.pose.position.y    = 0;
    pose_stamped.pose.position.z    = 0;
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1.;
    pose_publisher.publish(pose_stamped);

    ROS_INFO("DONE! Starting Localization and Mapping.");

    init = false;
  } else if (!init && !init_odom && (!init_gps || !params.use_gps)) {

    // --------- Build local maps to use in the localization
    // - Compute 2D local map of semantic features on robot's referential frame
    std::vector<SemanticFeature> m_landmarks;
    mapper2D->localMap(bearings, depths, m_landmarks);
    // - Compute 3D PCL corners and ground plane on robot's referential frame
    std::vector<Corner>  m_corners;
    std::vector<Cluster> m_clusters;
    std::vector<Line>    m_vegetation_lines;
    Plane                m_ground_plane;
    mapper3D->localPCLMap(
        scan_pts, m_corners, m_clusters, m_vegetation_lines, m_ground_plane);
    // - Compute 3D image features on robot's referential frame
    std::vector<ImageFeature> m_surf_features;
    mapper3D->localSurfMap(left_image, raw_depths, m_surf_features);

    // ------- Build observation structure to use in the localization
    Observation obsv;
    if (params.use_landmarks)
      obsv.landmarks = m_landmarks;
    obsv.corners          = m_corners;
    obsv.vegetation_lines = m_vegetation_lines;
    obsv.ground_plane     = m_ground_plane;
    if (has_converged && params.use_gps)
      obsv.gps_pose = gps_pose;
    else
      obsv.gps_pose = pose(0., 0., 0., 0., 0., 0.);
    obsv.surf_features = m_surf_features;

    // ------- LOCALIZATION PROCEDURE ---------- //
    localizer->process(odom, obsv, grid_map);
    robot_pose = localizer->getPose();

    if (register_map) {
      // ------- MULTI-LAYER MAPPING REGISTRATION ------------ //
      // - 2D high-level semantic map estimation
      mapper2D->process(robot_pose, m_landmarks, labels, *grid_map);
      // - 3D PCL corner map estimation
      mapper3D->globalCornerMap(robot_pose, m_corners, *grid_map);
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
    std::array<float, 9> odom_R{};
    odom.toRotMatrix(odom_R);
    TF odom_tf(odom_R, std::array<float, 3>{odom.x, odom.y, odom.z});
    odom_path.push_back(odom_tf);

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
    pose_stamped.header.stamp       = ros::Time::now();
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
    ros_path.header.stamp    = ros::Time::now();
    ros_path.header.frame_id = "map";
    ros_path.poses           = path;
    path_publisher.publish(ros_path);

    // Publish cam-to-map tf::Transform
    static tf::TransformBroadcaster br;
    br.sendTransform(
        tf::StampedTransform(base2map, ros::Time::now(), "map", "base_link"));
    // Publish other tf::Trasforms
    tf::Transform cam2base(
        tf::Quaternion(params.cam2base[3],
                       params.cam2base[4],
                       params.cam2base[5],
                       params.cam2base[6]),
        tf::Vector3(params.cam2base[0], params.cam2base[1], params.cam2base[2]));
    br.sendTransform(tf::StampedTransform(
        cam2base, ros::Time::now(), "base_link", "zed_camera_left_optical_frame"));
    tf::Transform vel2base(
        tf::Quaternion(params.vel2base[3],
                       params.vel2base[4],
                       params.vel2base[5],
                       params.vel2base[6]),
        tf::Vector3(params.vel2base[0], params.vel2base[1], params.vel2base[2]));
    br.sendTransform(
        tf::StampedTransform(vel2base, ros::Time::now(), "base_link", "velodyne"));

    // ---------- Publish Multi-layer map ------------- //
    // Publish the grid map
    publishGridMap(depth_image->header);
    // Publish the 2D map
    publish2DMap(depth_image->header, robot_pose, bearings, depths);
    // Publish 3D maps
    publish3DMap();
    publish3DMap(m_corners, corners_local_publisher);
    publish3DMap(m_vegetation_lines, map3D_lines_publisher);
    std::vector<Plane> planes = {m_ground_plane};
    publish3DMap(planes, map3D_planes_publisher);
    // ------------------------------------------------ //

    // --------------------------------------------------
    // ----- Debug area : publishes the robot path & the vegetation lines
    // --------------------------------------------------
    if (params.debug) {
      // Publish all poses for DEBUG
      // ----------------------------------------------------------------------------
      std::vector<pose> poses;
      (*localizer).getParticles(poses);
      geometry_msgs::PoseArray ros_poses;
      ros_poses.header.stamp    = ros::Time::now();
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

      // - Publish associations between corners
      visualization_msgs::MarkerArray markers;

      // - Publish vegetation lines for debug
      if (m_vegetation_lines.size() == 2) {
        geometry_msgs::Point P1_a, P2_a, P1_b, P2_b;
        float                x_min_a = m_vegetation_lines[0].pts[0].x;
        float                x_max_a =
            m_vegetation_lines[0].pts[m_vegetation_lines[0].pts.size() - 1].x;
        float x_min_b = m_vegetation_lines[1].pts[0].x;
        float x_max_b =
            m_vegetation_lines[1].pts[m_vegetation_lines[1].pts.size() - 1].x;
        P1_a.x = x_min_a;
        P1_a.y = x_min_a * m_vegetation_lines[0].m + m_vegetation_lines[0].b;
        P1_a.z = 0;
        P2_a.x = x_max_a;
        P2_a.y = x_max_a * m_vegetation_lines[0].m + m_vegetation_lines[0].b;
        P2_a.z = 0;
        P1_b.x = x_min_b;
        P1_b.y = x_min_b * m_vegetation_lines[1].m + m_vegetation_lines[1].b;
        P1_b.z = 0;
        P2_b.x = x_max_b;
        P2_b.y = x_max_b * m_vegetation_lines[1].m + m_vegetation_lines[1].b;
        P2_b.z = 0;

        visualization_msgs::Marker marker_a;
        marker_a.header.frame_id = "base_link";
        marker_a.header.stamp    = ros::Time::now();
        marker_a.ns              = "line_a";
        marker_a.id              = 0;
        marker_a.type            = visualization_msgs::Marker::LINE_STRIP;
        marker_a.action          = visualization_msgs::Marker::ADD;
        marker_a.points.push_back(P1_a);
        marker_a.points.push_back(P2_a);
        marker_a.color.a = 1;
        marker_a.color.r = 1;
        marker_a.color.b = 0;
        marker_a.color.g = 0;
        marker_a.scale.x = 0.1;
        marker_a.scale.y = 0.1;
        marker_a.scale.z = 0.1;
        visualization_msgs::Marker marker_b;
        marker_b.header.frame_id = "base_link";
        marker_b.header.stamp    = ros::Time::now();
        marker_b.ns              = "line_b";
        marker_b.id              = 1;
        marker_b.type            = visualization_msgs::Marker::LINE_STRIP;
        marker_b.action          = visualization_msgs::Marker::ADD;
        marker_b.points.push_back(P1_b);
        marker_b.points.push_back(P2_b);
        marker_b.color.a = 1;
        marker_b.color.r = 0;
        marker_b.color.b = 1;
        marker_b.color.g = 0;
        marker_b.scale.x = 0.1;
        marker_b.scale.y = 0.1;
        marker_b.scale.z = 0.1;
        markers.markers.push_back(marker_a);
        markers.markers.push_back(marker_b);
      }

      for (const auto& cluster : m_clusters) {
        point m_pt;
        m_pt.x = cluster.center.x * robot_tf.R[0] +
                 cluster.center.y * robot_tf.R[1] +
                 cluster.center.z * robot_tf.R[2] + robot_tf.t[0];
        m_pt.y = cluster.center.x * robot_tf.R[3] +
                 cluster.center.y * robot_tf.R[4] +
                 cluster.center.z * robot_tf.R[5] + robot_tf.t[1];
        m_pt.z = cluster.center.x * robot_tf.R[6] +
                 cluster.center.y * robot_tf.R[7] +
                 cluster.center.z * robot_tf.R[8] + robot_tf.t[2];

        point radius = cluster.radius;
        if (cluster.items.size() == 1)
          radius = point(0.06, 0.06, 0.06);

        visualization_msgs::Marker ros_cluster;
        ros_cluster.header.frame_id    = "map";
        ros_cluster.header.stamp       = ros::Time::now();
        ros_cluster.ns                 = "sphere_" + std::to_string(cluster.id);
        ros_cluster.type               = visualization_msgs::Marker::CUBE;
        ros_cluster.action             = visualization_msgs::Marker::ADD;
        ros_cluster.pose.position.x    = m_pt.x;
        ros_cluster.pose.position.y    = m_pt.y;
        ros_cluster.pose.position.z    = m_pt.z;
        ros_cluster.pose.orientation.x = 0;
        ros_cluster.pose.orientation.y = 0;
        ros_cluster.pose.orientation.z = 0;
        ros_cluster.pose.orientation.w = 1;
        ros_cluster.color.a            = 0.5;
        ros_cluster.color.r            = 0.5;
        ros_cluster.color.b            = 1;
        ros_cluster.color.g            = 0.5;
        ros_cluster.scale.x            = radius.x * 2;
        ros_cluster.scale.y            = radius.y * 2;
        ros_cluster.scale.z            = radius.z * 2;

        markers.markers.push_back(ros_cluster);
      }
      debug_markers.publish(markers);
    }
  }
}

void VineSLAM_ros::scanListener(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_pcl(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *velodyne_pcl);
  // Remove Nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*velodyne_pcl, *velodyne_pcl, indices);

  std::vector<float> intensities;
  scan_pts.clear();
  for (const auto& pt : *velodyne_pcl) {
    point m_pt(pt.x, pt.y, pt.z);
    scan_pts.push_back(m_pt);
    intensities.push_back(pt.intensity);
  }
}

void VineSLAM_ros::odomListener(const nav_msgs::OdometryConstPtr& msg)
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

void VineSLAM_ros::gpsListener(const sensor_msgs::NavSatFixConstPtr& msg)
{
  if (!params.use_gps)
    return;

  if (init_gps) {
    has_converged = false;

    // Set initial datum
    agrob_map_transform::SetDatum srv;
    srv.request.geo_pose.position.latitude  = params.latitude;
    srv.request.geo_pose.position.longitude = params.longitude;
    srv.request.geo_pose.position.altitude  = 0.0;
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(quat, srv.request.geo_pose.orientation);

    ROS_INFO("Setting GNSS datum...");
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

    if (estimate_heading)
      has_converged = getGNSSHeading(gps_odom, msg->header);

    // Compute the gnss to map transform
    tf::Quaternion heading_quat;
    heading_quat.setRPY(0., 0., heading);
    heading_quat.normalize();
    tf::Transform ned2map(heading_quat, tf::Vector3(0., 0., 0.));

    // Publish gnss to map tf::Transform
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(ned2map, ros::Time::now(), "enu", "map"));

    // Publish gnss pose in the enu reference frame
    geometry_msgs::PoseStamped gnss_pose;
    gnss_pose.pose.position.x    = gps_odom.x;
    gnss_pose.pose.position.y    = gps_odom.y;
    gnss_pose.pose.position.z    = gps_odom.z;
    gnss_pose.pose.orientation.x = 0.;
    gnss_pose.pose.orientation.y = 0.;
    gnss_pose.pose.orientation.z = 0.;
    gnss_pose.pose.orientation.w = 1.;
    gnss_pose.header.stamp       = ros::Time::now();
    gnss_pose.header.frame_id    = "enu";

    gps_poses.push_back(gnss_pose);
    gps_publisher.publish(gps_poses);

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

bool VineSLAM_ros::getGNSSHeading(const pose&             gps_odom,
                                  const std_msgs::Header& header)
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

  return weight_max > 0.6;
}

void VineSLAM_ros::computeObsv(const sensor_msgs::Image& depth_img,
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
        float y         = -(static_cast<float>(i) - params.cx) * (x / params.fx);
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

void VineSLAM_ros::publishGridMap(const std_msgs::Header& header) const
{
  // Define ROS occupancy grid map
  nav_msgs::OccupancyGrid occ_map;
  occ_map.header.stamp    = ros::Time::now();
  occ_map.header.frame_id = "map";

  // Set the map metadata
  nav_msgs::MapMetaData metadata;
  metadata.origin.position.x    = params.gridmap_origin_x;
  metadata.origin.position.y    = params.gridmap_origin_y;
  metadata.origin.position.z    = 0;
  metadata.origin.orientation.x = 0.;
  metadata.origin.orientation.y = 0.;
  metadata.origin.orientation.z = 0.;
  metadata.origin.orientation.w = 1.;
  metadata.resolution           = params.gridmap_resolution;
  metadata.width                = params.gridmap_width / params.gridmap_resolution;
  metadata.height               = params.gridmap_height / params.gridmap_resolution;
  occ_map.info                  = metadata;

  // Fill the occupancy grid map
  occ_map.data.resize(metadata.width * metadata.height);
  // - compute x and y bounds
  int xmin = static_cast<int>(params.gridmap_origin_x / params.gridmap_resolution);
  int xmax = static_cast<int>((float)xmin +
                              params.gridmap_width / params.gridmap_resolution - 1);
  int ymin = static_cast<int>(params.gridmap_origin_y / params.gridmap_resolution);
  int ymax = static_cast<int>((float)ymin +
                              params.gridmap_height / params.gridmap_resolution - 1);
  for (int i = xmin; i < xmax; i++) {
    for (int j = ymin; j < ymax; j++) {
      int8_t number_objs = (*grid_map)(i, j).landmarks.size() +
                           (*grid_map)(i, j).surf_features.size() +
                           (*grid_map)(i, j).corner_features.size();

      int m_i =
          i - static_cast<int>(params.gridmap_origin_x / params.gridmap_resolution);
      int m_j =
          j - static_cast<int>(params.gridmap_origin_y / params.gridmap_resolution);
      int idx = m_i + m_j * static_cast<int>(
                                (params.gridmap_width / params.gridmap_resolution));

      occ_map.data[idx] = number_objs * 10;
    }
  }

  // Publish the map
  mapOCC_publisher.publish(occ_map);
}

void VineSLAM_ros::publish2DMap(const std_msgs::Header&   header,
                                const pose&               pose,
                                const std::vector<float>& bearings,
                                const std::vector<float>& depths) const
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
      marker.header.stamp    = ros::Time::now();
      marker.header.frame_id = "map";
      marker.pose.position.x = m_sfeature.second.pos.x;
      marker.pose.position.y = m_sfeature.second.pos.y;
      marker.pose.position.z = m_sfeature.second.pos.z;

      marker_array.markers.push_back(marker);

      // Draw sfeature standard deviation
      tf2::Quaternion q;
      q.setRPY(0, 0, m_sfeature.second.gauss.theta);

      ellipse.id                 = id;
      ellipse.header.stamp       = ros::Time::now();
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
  ellipse.header.stamp       = ros::Time::now();
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

void VineSLAM_ros::publish3DMap() const
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
      pcl::PointXYZI m_pt(static_cast<float>(corner.which_cluster));
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

void VineSLAM_ros::publish3DMap(const std::vector<Plane>& planes,
                                const ros::Publisher&     pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  int i = 0;
  for (const auto& plane : planes) {
    for (const auto& pt : plane.points) {
      std::array<float, 9> robot_R{};
      robot_pose.toRotMatrix(robot_R);
      TF robot_tf(robot_R,
                  std::array<float, 3>{robot_pose.x, robot_pose.y, robot_pose.z});

      pcl::PointXYZI m_pt(i);
      m_pt.x = pt.x * robot_tf.R[0] + pt.y * robot_tf.R[1] + pt.z * robot_tf.R[2] +
               robot_tf.t[0];
      m_pt.y = pt.x * robot_tf.R[3] + pt.y * robot_tf.R[4] + pt.z * robot_tf.R[5] +
               robot_tf.t[1];
      m_pt.z = pt.x * robot_tf.R[6] + pt.y * robot_tf.R[7] + pt.z * robot_tf.R[8] +
               robot_tf.t[2];

      cloud_out->points.push_back(m_pt);
    }
    i++;
  }

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const std::vector<Line>& vegetation_lines,
                                const ros::Publisher&    pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  int i = 0;
  for (const auto& line : vegetation_lines) {
    for (const auto& pt : line.pts) {
      std::array<float, 9> robot_R{};
      robot_pose.toRotMatrix(robot_R);
      TF robot_tf(robot_R,
                  std::array<float, 3>{robot_pose.x, robot_pose.y, robot_pose.z});

      pcl::PointXYZI m_pt(i);
      m_pt.x = pt.x * robot_tf.R[0] + pt.y * robot_tf.R[1] + pt.z * robot_tf.R[2] +
               robot_tf.t[0];
      m_pt.y = pt.x * robot_tf.R[3] + pt.y * robot_tf.R[4] + pt.z * robot_tf.R[5] +
               robot_tf.t[1];
      m_pt.z = pt.x * robot_tf.R[6] + pt.y * robot_tf.R[7] + pt.z * robot_tf.R[8] +
               robot_tf.t[2];

      cloud_out->points.push_back(m_pt);
    }
    i++;
  }

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

void VineSLAM_ros::publish3DMap(const std::vector<Corner>& corners,
                                const ros::Publisher&      pub)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointXYZI m_pt;

  for (const auto& corner : corners) {
    std::array<float, 9> robot_R{};
    robot_pose.toRotMatrix(robot_R);
    TF robot_tf(robot_R,
                std::array<float, 3>{robot_pose.x, robot_pose.y, robot_pose.z});

    m_pt.x = corner.pos.x * robot_tf.R[0] + corner.pos.y * robot_tf.R[1] +
             corner.pos.z * robot_tf.R[2] + robot_tf.t[0];
    m_pt.y = corner.pos.x * robot_tf.R[3] + corner.pos.y * robot_tf.R[4] +
             corner.pos.z * robot_tf.R[5] + robot_tf.t[1];
    m_pt.z = corner.pos.x * robot_tf.R[6] + corner.pos.y * robot_tf.R[7] +
             corner.pos.z * robot_tf.R[8] + robot_tf.t[2];

    m_pt.intensity = static_cast<float>(corner.which_cluster) * 10.0f;
    cloud_out->points.push_back(m_pt);
  }

  cloud_out->header.frame_id = "map";
  pub.publish(cloud_out);
}

} // namespace vineslam
