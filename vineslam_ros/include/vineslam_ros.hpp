#pragma once

// vineslam members
#include <vineslam/feature/semantic.hpp>
#include <vineslam/feature/visual.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/localization/localizer.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/mapping/elevation_map.hpp>
#include <vineslam/mapping/landmark_mapping.hpp>
#include <vineslam/mapping/visual_mapping.hpp>
#include <vineslam/mapping/lidar_mapping.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Const.hpp>
#include <vineslam/math/Geodetic.hpp>
#include <vineslam/map_io/map_writer.hpp>
#include <vineslam/map_io/map_parser.hpp>
#include <vineslam/map_io/elevation_map_writer.hpp>
#include <vineslam/map_io/elevation_map_parser.hpp>
#include <vineslam/utils/save_data.hpp>
#include <vineslam/utils/Timer.hpp>
// ----------------------------
#include <vineslam_msgs/particle.h>
#include <vineslam_msgs/report.h>
#include <vineslam_msgs/Feature.h>
#include <vineslam_msgs/FeatureArray.h>
// ----------------------------

// std
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <thread>

// ROS
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define LIDAR_SENSOR 0  // 0: velodyne, 1: livox

namespace vineslam
{
class VineSLAM_ros
{
public:
  VineSLAM_ros() = default;
  VineSLAM_ros(const std::string& node)
  {
  }

  // Stereo camera images callback function
  void imageFeatureListener(const vineslam_msgs::FeatureArrayConstPtr& features);

  // Landmark detection callback function
  void landmarkListener(const vision_msgs::Detection3DArrayConstPtr& dets);

  // Scan callback function
  void scanListener(const sensor_msgs::PointCloud2ConstPtr& msg);

  // Odometry callback function
  void odomListener(const nav_msgs::OdometryConstPtr& msg);

  // GPS callback function
  void gpsListener(const sensor_msgs::NavSatFixConstPtr& msg);
  //  void gpsListener(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  // IMU callback functions
  void imuListener(const geometry_msgs::Vector3StampedConstPtr& msg);
  void imuDataListener(const sensor_msgs::ImuConstPtr& msg);

  // GNSS heading estimation
  void getGNSSHeading();

  // Compute the motion increment by fusing different sources of information
  void computeInnovation(const Pose& wheel_odom_inc, const Pose& imu_rot_inc, Pose& output_pose);

  // Most recent message header received
  std_msgs::Header header_;

  // Tf2 broadcaster
  tf2_ros::TransformBroadcaster* tf_broadcaster_;
  tf2_ros::TransformListener* tf_listener_;
  tf2_ros::Buffer* tf_buffer_;

  // Global thread to publish maps and other info
  void publishDenseInfo(const float& rate);
  // Publish 2D semantic features map
  void publish2DMap() const;
  // Publish the elevation map
  void publishElevationMap() const;
  // Publish the 3D maps
  void publish3DMap();
  // Publish the 3D PCL planes
  void publish3DMap(const std::vector<Plane>& planes, const ros::Publisher& pub);
  void publish3DMap(const Pose& r_pose, const std::vector<Plane>& planes, const ros::Publisher& pub);
  // Publish the 3D PCL semi planes
  void publish3DMap(const std::vector<SemiPlane>& planes, const ros::Publisher& pub);
  void publish3DMap(const Pose& r_pose, const std::vector<SemiPlane>& planes, const ros::Publisher& pub);
  // Publish a 3D PCL corners map
  void publish3DMap(const std::vector<Corner>& corners, const ros::Publisher& pub);
  void publish3DMap(const Pose& r_pose, const std::vector<Corner>& corners, const ros::Publisher& pub);
  // Publish a 3D PCL planar features map
  void publish3DMap(const std::vector<Planar>& planars, const ros::Publisher& pub);
  void publish3DMap(const Pose& r_pose, const std::vector<Planar>& planars, const ros::Publisher& pub);
  // Creates a 6-DoF interactive marker
  void make6DofMarker(visualization_msgs::InteractiveMarker& imarker, Pose pose, std::string marker_name);
  // Publishes a box containing the grid map
  void publishGridMapLimits() const;
  // Publishes a box containing the zone occupied by the robot
  void publishRobotBox(const Pose& robot_pose) const;
  // Publishes a VineSLAM state report for debug purposes
  void publishReport() const;

  // VineSLAM input data
  struct InputData
  {
    // Landmark labels array
    std::vector<int> land_labels_;
    // Landmark bearings array
    std::vector<float> land_bearings_;
    // Landmark depths array
    std::vector<float> land_depths_;
    // Image features
    std::vector<ImageFeature> image_features_;
    // Wheel odometry pose
    Pose wheel_odom_pose_;
    // Previous wheel odometry pose
    Pose p_wheel_odom_pose_;
    // GNSS pose
    Pose gnss_pose_;
    Pose gnss_raw_pose_;
    // IMU poses
    Pose imu_pose_;
    Pose imu_data_pose_;

    // LiDAR scan points
    std::vector<Point> scan_pts_;

    // Observation flags
    bool received_landmarks_;
    bool received_image_features_;
    bool received_odometry_;
    bool received_gnss_;
    bool received_scans_;
  } input_data_;

  // ROS publishers/services
  ros::Publisher vineslam_report_publisher_;
  ros::Publisher grid_map_publisher_;
  ros::Publisher elevation_map_publisher_;
  ros::Publisher map2D_publisher_;
  ros::Publisher robot_box_publisher_;
  ros::Publisher map3D_features_publisher_;
  ros::Publisher map3D_corners_publisher_;
  ros::Publisher map3D_planars_publisher_;
  ros::Publisher map3D_planes_publisher_;
  ros::Publisher pose_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher poses_publisher_;
  ros::Publisher corners_local_publisher_;
  ros::Publisher planars_local_publisher_;
  ros::Publisher planes_local_publisher_;
  ros::Publisher gps_pose_publisher_;
  ros::Publisher gps_fix_publisher_;

  // Classes object members
  Parameters params_;
  Localizer* localizer_;
  ElevationMap* elevation_map_;
  OccupancyMap* grid_map_;
  LandmarkMapper* land_mapper_;
  VisualMapper* vis_mapper_;
#if LIDAR_SENSOR == 0
  VelodyneMapper* lid_mapper_;
#elif LIDAR_SENSOR == 1
  LivoxMapper* lid_mapper_;
#endif
  Timer* timer_;
  Geodetic* geodetic_converter_;
  Observation obsv_;

  // Array of poses to store and publish the robot path
  std::vector<geometry_msgs::PoseStamped> path_;

  // Motion variables
  Pose init_odom_pose_;
  Pose init_gps_pose_;
  Pose robot_pose_;

  // Timestamps
  std::chrono::time_point<std::chrono::high_resolution_clock> p_imu_observation_timestamp_;

  // Satellite -> map compensation
  bool estimate_heading_;

  // Initialization flags
  bool init_flag_;
  bool init_gps_;
  bool init_odom_;

  // Logs vars
  uint32_t n_saved_logs_;
  Pose p_saved_pose_;
  std::ofstream logs_file_;
};

}  // namespace vineslam
