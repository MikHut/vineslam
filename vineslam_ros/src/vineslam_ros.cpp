#include "../include/vineslam_ros.hpp"

namespace vineslam
{

void vineslam_ros::odomListener(const nav_msgs::OdometryConstPtr& msg)
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
  if (init) {
    p_odom.x   = (*msg).pose.pose.position.x;
    p_odom.y   = (*msg).pose.pose.position.y;
    p_odom.yaw = yaw;
    odom       = vineslam::pose(0., 0., 0., 0., 0., yaw);
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

void vineslam_ros::callbackFct(const sensor_msgs::ImageConstPtr& left_image,
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
  pose                      robot_pose;

  if (init && bearings.size() > 1) {
    // Initialize the localizer and get first particles distribution
    localizer->init(pose(0, 0, 0, 0, 0, odom.yaw));
    robot_pose = localizer->getPose();

    // Initialize the multi-layer map
    mapper2D->init(robot_pose, bearings, depths, labels, *grid_map);
    // MISSING 3D MAPS ...

    init = false;
  } else if (!init) {

    // --------- Build local maps to use in the localization
    // - Compute 2D local map of semantic features on robot's referential frame
    std::vector<SemanticFeature> m_landmarks;
    vineslam::Mapper2D::localMap(bearings, depths, m_landmarks);
    // - Compute 3D PCL corners and ground plane on robot's referential frame
    std::vector<Corner> m_corners;
    Plane               m_ground_plane;
    mapper3D->localPCLMap(raw_depths, m_corners, m_ground_plane);
    // MISSING feature 3D map

    // ------- Build observation structure to use in the localization
    Observation obsv;
    obsv.landmarks    = m_landmarks;
    obsv.corners      = m_corners;
    obsv.ground_plane = m_ground_plane;
    // MISSING 3D image features

    // ------- LOCALIZATION PROCEDURE ---------- //
    localizer->process(odom, obsv, *grid_map);
    robot_pose = localizer->getPose();

    // ------- MULTI-LAYER MAPPING ------------ //
    // ---------------------------------------- //
    // - 2D high-level semantic map estimation
    mapper2D->process(robot_pose, bearings, depths, labels, *grid_map);
    // - 3D PCL corner map estimation
    mapper3D->globalCornerMap(m_corners, robot_pose, *grid_map);
    // - MISSING 3D feature map ...
    // ---------------------------------------- //

    // Convert robot pose to tf::Transform corresponding
    // to the camera to map transformation
    tf::Quaternion q;
    q.setRPY(robot_pose.roll, robot_pose.pitch, robot_pose.yaw);
    q.normalize();
    tf::Transform cam2map;
    cam2map.setRotation(q);
    cam2map.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, robot_pose.z));

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
    br.sendTransform(
        tf::StampedTransform(cam2map, pose_stamped.header.stamp, "map", "cam"));

    // ---------- Publish Multi-layer map ------------- //
    // Publish the grid map
    publishGridMap(depth_image->header);
    // Publish the 2D map
    publish2DMap(depth_image->header, robot_pose, bearings, depths);
    // Publish 3D maps
    publish3DMap();
    //    publish3DMap(obsv.ground_plane, map3D_planes_publisher);
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
      tf::Quaternion q;
      q.setRPY(pose.roll, pose.pitch, pose.yaw);
      q.normalize();

      geometry_msgs::Pose m_pose;
      m_pose.position.x    = pose.x;
      m_pose.position.y    = pose.y;
      m_pose.position.z    = pose.z;
      m_pose.orientation.x = q.x();
      m_pose.orientation.y = q.y();
      m_pose.orientation.z = q.z();
      m_pose.orientation.w = q.w();

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

    Plane m_plane;
    for (const auto& pt : obsv.ground_plane.points) {
      point m_pt;
      m_pt.x = pt.x * R[0] + pt.y * R[1] + pt.z * R[2];
      m_pt.y = pt.x * R[3] + pt.y * R[4] + pt.z * R[5];
      m_pt.z = pt.x * R[6] + pt.y * R[7] + pt.z * R[8];

      m_plane.points.push_back(m_pt);
    }
    vector3D new_normal;
    new_normal.x   = m_normal.x * R[0] + m_normal.y * R[1] + m_normal.z * R[2];
    new_normal.y   = m_normal.x * R[3] + m_normal.y * R[4] + m_normal.z * R[5];
    new_normal.z   = m_normal.x * R[6] + m_normal.y * R[7] + m_normal.z * R[8];
    m_plane.normal = new_normal;
    publish3DMap(m_plane, map3D_planes_publisher);

    // - Publish ground plane normal for DEBUG
    float x = 0., y = 0., z = 0.;
    float min_x = 1000., min_y = 1000.;
    for (const auto& m_pt : m_plane.points) {
      if (x < min_x && y < min_y) {
        x = m_pt.x;
        y = m_pt.y;
        z = m_pt.z;

        min_x = x;
        min_y = y;
      }
    }
    geometry_msgs::Point p1, p2;
    p1.x = x;
    p1.y = y;
    p1.z = z;
    p2.x = p1.x + m_plane.normal.x;
    p2.y = p1.y + m_plane.normal.y;
    p2.z = p1.z + m_plane.normal.z;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp    = ros::Time();
    marker.ns              = "normal";
    marker.id              = 0;
    marker.type            = visualization_msgs::Marker::LINE_STRIP;
    marker.action          = visualization_msgs::Marker::ADD;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.color.a = 1;
    marker.color.r = 1;
    marker.color.b = 0;
    marker.color.g = 0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    normal_pub.publish(marker);

#endif
  }
}

void vineslam_ros::computeObsv(const sensor_msgs::Image& depth_img,
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

} // namespace vineslam
