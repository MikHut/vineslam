#include "../include/vineslam_ros.hpp"
#include "../include/convertions.hpp"

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Callbacks and observation functions
// --------------------------------------------------------------------------------

void VineSLAM_ros::scanListener(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_pcl(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *velodyne_pcl);
  // Remove Nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*velodyne_pcl, *velodyne_pcl, indices);

  input_data_.scan_pts_.clear();
  for (const auto& pt : *velodyne_pcl)
  {
    Point l_pt(pt.x, pt.y, pt.z, pt.intensity);
    input_data_.scan_pts_.push_back(l_pt);
  }

  input_data_.received_scans_ = true;
}

void VineSLAM_ros::odomListener(const nav_msgs::OdometryConstPtr& msg)
{
  header_ = msg->header;

  // If it is the first iteration - initialize odometry origin
  if (init_odom_)
  {
    // Convert odometry msg to pose msg
    tf2::Quaternion q;
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    q.setW(msg->pose.pose.orientation.w);
    q.normalize();

    // Check if yaw is NaN
    auto yaw = static_cast<float>(tf2::getYaw(q));
    if (!std::isfinite(yaw))
      yaw = 0;

    init_odom_pose_ = Pose(msg->pose.pose.position.x, msg->pose.pose.position.y, 0, 0, 0, yaw);
    input_data_.p_wheel_odom_pose_ = Pose(0, 0, 0, 0, 0, 0);

    init_odom_ = false;
    return;
  }

  // Transform odometry msg to maps' referential frame
  tf2::Quaternion o2m_q;
  o2m_q.setRPY(init_odom_pose_.R_, init_odom_pose_.P_, init_odom_pose_.Y_);
  tf2::Transform odom2map(o2m_q, tf2::Vector3(init_odom_pose_.x_, init_odom_pose_.y_, init_odom_pose_.z_));

  tf2::Quaternion odom_q;
  odom_q.setX(msg->pose.pose.orientation.x);
  odom_q.setY(msg->pose.pose.orientation.y);
  odom_q.setZ(msg->pose.pose.orientation.z);
  odom_q.setW(msg->pose.pose.orientation.w);
  tf2::Transform odom_tf(odom_q,
                         tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

  odom_tf = odom2map.inverseTimes(odom_tf);

  tf2::Vector3 trans = odom_tf.getOrigin();
  tf2::Quaternion rot = odom_tf.getRotation();

  input_data_.wheel_odom_pose_ = Pose(trans.x(), trans.y(), 0, 0, 0, static_cast<float>(tf2::getYaw(rot)));

  input_data_.received_odometry_ = true;
}

void VineSLAM_ros::gpsListener(const sensor_msgs::NavSatFixConstPtr& msg)
{
  header_ = msg->header;

  if (init_gps_)
  {
    geodetic_converter_ = new Geodetic(params_.map_datum_lat_, params_.map_datum_long_, params_.map_datum_alt_);
    init_gps_ = false;
  }
  else
  {
    double e, n, u;
    geodetic_converter_->geodetic2enu(msg->latitude, msg->longitude, msg->altitude, e, n, u);

    // Save pose
    input_data_.gnss_raw_pose_.x_ = n;
    input_data_.gnss_raw_pose_.y_ = -e;
    input_data_.gnss_raw_pose_.z_ = u;

    if (estimate_heading_)
    {
      getGNSSHeading();
    }

    // Convert the gnss with the correct heading
    Pose heading_pose(0, 0, 0, 0, 0, params_.map_datum_head_);
    Tf heading_tf = heading_pose.toTf();
    Point corrected_gnss_pose =
        Point(input_data_.gnss_raw_pose_.x_, input_data_.gnss_raw_pose_.y_, input_data_.gnss_raw_pose_.z_) *
        heading_tf.inverse();
    input_data_.gnss_pose_.x_ = corrected_gnss_pose.x_;
    input_data_.gnss_pose_.y_ = corrected_gnss_pose.y_;
    input_data_.gnss_pose_.z_ = corrected_gnss_pose.z_;

    // Set received flag to true
    input_data_.received_gnss_ = true;

    // Publish gps pose
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = header_.stamp;
    pose_stamped.header.frame_id = params_.world_frame_id_;
    pose_stamped.pose.position.x = input_data_.gnss_pose_.x_;
    pose_stamped.pose.position.y = input_data_.gnss_pose_.y_;
    pose_stamped.pose.position.z = input_data_.gnss_pose_.z_;
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1;
    gps_pose_publisher_.publish(pose_stamped);
  }
}

void VineSLAM_ros::imuListener(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  input_data_.imu_pose_.x_ = 0;
  input_data_.imu_pose_.y_ = 0;
  input_data_.imu_pose_.z_ = 0;
  input_data_.imu_pose_.R_ = msg->vector.x;
  input_data_.imu_pose_.P_ = msg->vector.y;
  input_data_.imu_pose_.Y_ = 0;
}

void VineSLAM_ros::imuDataListener(const sensor_msgs::ImuConstPtr& msg)
{
  auto c_imu_observation_timestamp = std::chrono::high_resolution_clock::now();

  if (!init_flag_)
  {
    double d = std::chrono::duration_cast<std::chrono::milliseconds>(c_imu_observation_timestamp -
                                                                     p_imu_observation_timestamp_)
                   .count();

    input_data_.imu_data_pose_.x_ = 0;
    input_data_.imu_data_pose_.y_ = 0;
    input_data_.imu_data_pose_.z_ = 0;
    input_data_.imu_data_pose_.R_ += msg->angular_velocity.x * (d / 1000);
    input_data_.imu_data_pose_.P_ += msg->angular_velocity.y * (d / 1000);
    input_data_.imu_data_pose_.Y_ += msg->angular_velocity.z * (d / 1000);
  }

  p_imu_observation_timestamp_ = c_imu_observation_timestamp;
}

void VineSLAM_ros::computeInnovation(const Pose& wheel_odom_inc, const Pose& imu_rot_inc, Pose& output_pose)
{
  double diff_yaw = std::fabs(wheel_odom_inc.Y_ - imu_rot_inc.Y_);
  double imu_weight = 0;
  if (diff_yaw < (0.5 * DEGREE_TO_RAD))
  {
    imu_weight = 0.5;
  }
  else if (diff_yaw > (0.5 * DEGREE_TO_RAD) && diff_yaw < (1.5 * DEGREE_TO_RAD))
  {
    imu_weight = 0.8;
  }
  else
  {
    imu_weight = 1.0;
  }

  double s_odom_Y = std::sin(wheel_odom_inc.Y_) * (1 - imu_weight);
  double c_odom_Y = std::cos(wheel_odom_inc.Y_) * (1 - imu_weight);
  double s_imu_Y = std::sin(imu_rot_inc.Y_) * imu_weight;
  double c_imu_Y = std::cos(imu_rot_inc.Y_) * imu_weight;
  double hat_yaw = std::atan2(s_odom_Y + s_imu_Y, c_odom_Y + c_imu_Y);

  output_pose = Pose(wheel_odom_inc.x_, wheel_odom_inc.y_, 0, imu_rot_inc.R_, imu_rot_inc.P_, hat_yaw);
}

void VineSLAM_ros::getGNSSHeading()
{
  float robot_distance_traveleld = robot_pose_.norm3D();

  if (robot_distance_traveleld > 0.1 && robot_distance_traveleld < 5.0)
  {
    float phi = 0.;
    float min_dist = std::numeric_limits<float>::max();
    while (phi < 360.)
    {
      float x = std::cos(phi * DEGREE_TO_RAD) * robot_pose_.x_ - std::sin(phi * DEGREE_TO_RAD) * robot_pose_.y_;
      float y = std::sin(phi * DEGREE_TO_RAD) * robot_pose_.x_ + std::cos(phi * DEGREE_TO_RAD) * robot_pose_.y_;

      Point source(x, y, 0.);
      Point target(input_data_.gnss_raw_pose_.x_, input_data_.gnss_raw_pose_.y_, 0.);
      float dist = source.distanceXY(target);

      if (dist < min_dist)
      {
        min_dist = dist;
        params_.map_datum_head_ = Const::normalizeAngle(phi * DEGREE_TO_RAD);
      }

      phi += 0.1;
    }
  }
  else if (robot_distance_traveleld >= 5.0)
  {
    localizer_->changeGPSFlag(true);  // Set the confidence of the use of gps in the particle filter now that we have
                                      // estimated heading.
    estimate_heading_ = false;
  }
}

}  // namespace vineslam