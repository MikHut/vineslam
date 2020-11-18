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

  // if ( ... ) {
  // -- here comes the initialization procedures
  // -- all the maps are initialized with the first observations
  // -- the robot pose is also initialized, with a gaussian distribution on the
  //    particle filter
  // } else if ( ... ) {
  // -- here is where all the runtime localization and mapping procedures take place
  // -- the multi-layer map is created
  // -- the particle filter is used to localize the robot
  // -- visualization functions are invocated to publish maps
  // }

  if (init && !init_odom && (!init_gps || !params.use_gps) &&
      (bearings.size() > 1 || !params.use_landmarks)) {

    // ---------------------------------------------------------
    // ----- Initialize the localizer and get first particles distribution
    // ---------------------------------------------------------
    localizer->init(pose(0, 0, 0, 0, 0, 0));
    robot_pose = localizer->getPose();
    grid_map   = new OccupancyMap(params, pose(0, 0, 0, 0, 0, 0));

    if (register_map) {
      // ---------------------------------------------------------
      // ----- Initialize the multi-layer maps
      // ---------------------------------------------------------

      // - 2D semantic feature map
      mapper2D->init(robot_pose, bearings, depths, labels, *grid_map);

      // - 3D PCL corner map estimation
      std::vector<Corner> m_corners;
      std::vector<Planar> m_planars;
      std::vector<Plane>  m_planes;
      Plane               m_ground_plane;
      mapper3D->localPCLMap(
          scan_pts, m_corners, m_planars, m_planes, m_ground_plane);

      // - 3D image feature map estimation
      auto*                     raw_depths = (float*)(&(*depth_image).data[0]);
      std::vector<ImageFeature> m_surf_features;
      mapper3D->localSurfMap(left_image, raw_depths, m_surf_features);

      // - Register 3D maps
      mapper3D->registerMaps(
          robot_pose, m_surf_features, m_corners, m_planars, m_planes, *grid_map);

      // - Save local map for next iteration
      previous_map->clear();
      for (const auto& planar : m_planars) previous_map->insert(planar);
      for (const auto& corner : m_corners) previous_map->insert(corner);
      previous_map->downsamplePlanars();
    }

    ROS_INFO("Localization and Mapping has started.");

    init = false;
  } else if (!init && !init_odom && (!init_gps || !params.use_gps)) {

    // ---------------------------------------------------------
    // ----- Build local maps to use in the localization
    // ---------------------------------------------------------
    // - Compute 2D local map of semantic features on robot's referential frame
    std::vector<SemanticFeature> m_landmarks;
    mapper2D->localMap(bearings, depths, m_landmarks);

    // - Compute 3D PCL corners and ground plane on robot's referential frame
    std::vector<Corner> m_corners;
    std::vector<Planar> m_planars;
    std::vector<Plane>  m_planes;
    Plane               m_ground_plane;
    mapper3D->localPCLMap(scan_pts, m_corners, m_planars, m_planes, m_ground_plane);

    // - Compute 3D image features on robot's referential frame
    std::vector<ImageFeature> m_surf_features;
    auto*                     raw_depths = (float*)(&(*depth_image).data[0]);
    mapper3D->localSurfMap(left_image, raw_depths, m_surf_features);

    // ---------------------------------------------------------
    // ----- Build observation structure to use in the localization
    // ---------------------------------------------------------
    // * High level landmarks (if we're using them)
    // * Point cloud corners
    // * Vegetation lines (if they we're detected and are parallel)
    // * Ground plane (if it is at the right distance to the LiDAR sensor)
    // * SURF 3D image features
    // * GPS (if we're using it)
    if (params.use_landmarks)
      obsv.landmarks = m_landmarks;
    if (params.use_corners)
      obsv.corners = m_corners;
    if (params.use_planars)
      obsv.planars = m_planars;
    if (params.use_planes)
      obsv.planes = m_planes;
    if (params.use_ground_plane)
      obsv.ground_plane = m_ground_plane;
    if (params.use_icp)
      obsv.surf_features = m_surf_features;
    if (has_converged && params.use_gps)
      obsv.gps_pose = gps_pose;
    else
      obsv.gps_pose = pose(0., 0., 0., 0., 0., 0.);

    // ---------------------------------------------------------
    // ----- Localization procedure
    // ---------------------------------------------------------
    localizer->process(odom, obsv, previous_map, grid_map);
    robot_pose = localizer->getPose();

    // ---------------------------------------------------------
    // ----- Register multi-layer map (if performing SLAM)
    // ---------------------------------------------------------
    if (register_map) {
      mapper2D->process(robot_pose, m_landmarks, labels, *grid_map);

      mapper3D->registerMaps(
          robot_pose, m_surf_features, m_corners, m_planars, m_planes, *grid_map);
      grid_map->downsamplePlanars();
    }

    // ---------------------------------------------------------
    // ----- ROS publishers and tf broadcasting
    // ---------------------------------------------------------

    // Convert robot pose to tf::Transform corresponding
    tf::Quaternion q;
    q.setRPY(robot_pose.roll, robot_pose.pitch, robot_pose.yaw);
    q.normalize();
    tf::Transform base2map;
    base2map.setRotation(q);
    base2map.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, robot_pose.z));

    // Publish tf::Trasforms
    static tf::TransformBroadcaster br;
    br.sendTransform(
        tf::StampedTransform(base2map, ros::Time::now(), "map", "base_link"));
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
    tf::Quaternion o2m_q;
    o2m_q.setRPY(init_odom_pose.roll, init_odom_pose.pitch, init_odom_pose.yaw);
    tf::Transform odom2map(
        o2m_q, tf::Vector3(init_odom_pose.x, init_odom_pose.y, init_odom_pose.z));
    br.sendTransform(
        tf::StampedTransform(odom2map, ros::Time::now(), "odom", "map"));

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

    // Publish particle poses (after and before resampling)
    // - Get the particles
    std::vector<Particle> b_particles, a_particles;
    (*localizer).getParticlesBeforeResampling(b_particles);
    (*localizer).getParticles(a_particles);
    // - Convert them to ROS pose array and fill the vineslam report msgs
    vineslam_msgs::report    report;
    geometry_msgs::PoseArray ros_poses;
    ros_poses.header.stamp    = ros::Time::now();
    ros_poses.header.frame_id = "map";
    report.header.stamp       = ros_poses.header.stamp;
    report.header.frame_id    = ros_poses.header.frame_id;
    for (const auto& particle : b_particles) {
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

      vineslam_msgs::particle particle_info;
      particle_info.id   = particle.id;
      particle_info.pose = m_pose;
      particle_info.w    = particle.w;

      report.b_particles.push_back(particle_info);
    }
    for (const auto& particle : a_particles) {
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

      vineslam_msgs::particle particle_info;
      particle_info.id   = particle.id;
      particle_info.pose = m_pose;
      particle_info.w    = particle.w;

      report.a_particles.push_back(particle_info);
    }
    poses_publisher.publish(ros_poses);

    report.log.data            = localizer->logs;
    report.use_high_level.data = params.use_landmarks;
    report.use_corners.data    = params.use_corners;
    report.use_planars.data    = params.use_planars;
    report.use_planes.data     = params.use_planes;
    report.use_icp.data        = params.use_icp;
    report.use_gps.data        = params.use_gps;
    vineslam_report_publisher.publish(report);

    // Publish the 2D map
    publish2DMap(depth_image->header, robot_pose, bearings, depths);
    // Publish 3D maps
    publish3DMap();
    publish3DMap(m_corners, corners_local_publisher);
    publish3DMap(m_planars, planars_local_publisher);
    std::vector<Plane> planes = {m_ground_plane};
    for (const auto& plane : m_planes) planes.push_back(plane);
    publish3DMap(planes, planes_local_publisher);

    // - Save local map for next iteration
    previous_map->clear();
    for (const auto& planar : m_planars) previous_map->insert(planar);
    for (const auto& corner : m_corners) previous_map->insert(corner);
    previous_map->downsamplePlanars();
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

  // If it is the first iteration - initialize odometry origin
  if (init_odom) {
    // Convert odometry msg to pose msg
    tf::Pose            pose_;
    geometry_msgs::Pose odom_pose = (*msg).pose.pose;
    tf::poseMsgToTF(odom_pose, pose_);

    // Check if yaw is NaN
    float yaw = static_cast<float>(tf::getYaw(pose_.getRotation()));
    if (!std::isfinite(yaw))
      yaw = 0;

    init_odom_pose =
        pose(msg->pose.pose.position.x, msg->pose.pose.position.y, 0, 0, 0, yaw);
    init_odom = false;

    return;
  }

  // Transform odometry msg to maps' referential frame
  tf::Quaternion o2m_q;
  o2m_q.setRPY(init_odom_pose.roll, init_odom_pose.pitch, init_odom_pose.yaw);
  tf::Transform odom2map(
      o2m_q, tf::Vector3(init_odom_pose.x, init_odom_pose.y, init_odom_pose.z));

  tf::Quaternion odom_q;
  odom_q.setX(msg->pose.pose.orientation.x);
  odom_q.setY(msg->pose.pose.orientation.y);
  odom_q.setZ(msg->pose.pose.orientation.z);
  odom_q.setW(msg->pose.pose.orientation.w);
  tf::Transform odom_tf(odom_q,
                        tf::Vector3(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z));

  odom_tf = odom2map.inverseTimes(odom_tf);

  tf::Vector3    trans = odom_tf.getOrigin();
  tf::Quaternion rot   = odom_tf.getRotation();

  odom = pose(trans.x(), trans.y(), 0, 0, 0, static_cast<float>(tf::getYaw(rot)));
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
    nav_msgs::Path ros_path;
    ros_path.header.stamp    = ros::Time::now();
    ros_path.header.frame_id = "map";
    ros_path.poses           = gps_poses;
    gps_publisher.publish(ros_path);

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
  for (uint32_t i = xmin; i < xmax; i++) {
    for (uint32_t j = ymin; j < ymax; j++) {
      uint32_t idx = i + depth_img.width * j;

      // Fill the depth array with the values of interest
      if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
          depths[idx] < range_max) {
        float x         = depths[idx];
        float y         = -(static_cast<float>(i) - params.cx) * (x / params.fx);
        auto  m_depth   = static_cast<float>(sqrt(pow(x, 2) + pow(y, 2)));
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

} // namespace vineslam
