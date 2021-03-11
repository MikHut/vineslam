#pragma once

// vineslam members
#include <vineslam/params.hpp>
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
#include <vineslam/math/const.hpp>
#include <vineslam/mapxml/map_writer.hpp>
#include <vineslam/mapxml/map_parser.hpp>
#include <vineslam/utils/save_data.hpp>
#include <vineslam/utils/Timer.hpp>
// ----------------------------
#include <vineslam_msgs/particle.h>
#include <vineslam_msgs/report.h>
#include <vineslam_msgs/Feature.h>
#include <vineslam_msgs/FeatureArray.h>
// ----------------------------
#include <vineslam_ros/start_map_registration.h>
#include <vineslam_ros/stop_map_registration.h>
#include <vineslam_ros/stop_gps_heading_estimation.h>
#include <vineslam_ros/save_map.h>
// ----------------------------

// std
#include <iostream>
#include <ctime>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>

// Services
#include <agrob_map_transform/GetPose.h>
#include <agrob_map_transform/SetDatum.h>

namespace vineslam
{
class VineSLAM_ros
{
public:
  VineSLAM_ros() = default;

  // Runtime execution routines
  virtual void init();
  virtual void loop();
  virtual void loopOnce();
  virtual void process();

  // Image features listener
  void imageFeatureListener(const vineslam_msgs::FeatureArrayConstPtr& features);
  // Landmark detection callback function
  void landmarkListener(const vision_msgs::Detection3DArrayConstPtr& dets);
  // Scan callback function
  void scanListener(const sensor_msgs::PointCloud2ConstPtr& msg);
  // Odometry callback function
  void odomListener(const nav_msgs::OdometryConstPtr& msg);
  // GPS callback function
  void gpsListener(const sensor_msgs::NavSatFixConstPtr& msg);
  // Services callbacks
  bool startRegistration(vineslam_ros::start_map_registration::Request&,
                         vineslam_ros::start_map_registration::Response&);
  bool stopRegistration(vineslam_ros::stop_map_registration::Request&, vineslam_ros::stop_map_registration::Response&);
  bool stopHeadingEstimation(vineslam_ros::stop_gps_heading_estimation::Request&,
                             vineslam_ros::stop_gps_heading_estimation::Response&);
  bool saveMap(vineslam_ros::save_map::Request&, vineslam_ros::save_map::Response&);

  // Publish 2D semantic features map
  void publish2DMap(const Pose& pose, const std::vector<float>& bearings, const std::vector<float>& depths) const;
  // Publish the elevation map
  void publishElevationMap() const;
  // Publish the 3D maps
  void publish3DMap() const;
  // Publish the 3D PCL planes
  void publish3DMap(const std::vector<Plane>& planes, const ros::Publisher& pub);
  static void publish3DMap(const Pose& r_pose, const std::vector<Plane>& planes, const ros::Publisher& pub);
  // Publish the 3D PCL semi planes
  void publish3DMap(const std::vector<SemiPlane>& planes, const ros::Publisher& pub);
  static void publish3DMap(const Pose& r_pose, const std::vector<SemiPlane>& planes, const ros::Publisher& pub);
  // Publish a 3D PCL corners map
  void publish3DMap(const std::vector<Corner>& corners, const ros::Publisher& pub);
  static void publish3DMap(const Pose& r_pose, const std::vector<Corner>& corners, const ros::Publisher& pub);
  // Publish a 3D PCL planar features map
  void publish3DMap(const std::vector<Planar>& planars, const ros::Publisher& pub);
  static void publish3DMap(const Pose& r_pose, const std::vector<Planar>& planars, const ros::Publisher& pub);
  // Publish the grid map that contains all the maps
  void publishGridMap(const std_msgs::Header& header) const;
  // Publishes a VineSLAM state report for debug purposes
  void publishReport() const;

  // GNSS heading estimator
  bool getGNSSHeading(const Pose& gps_odom, const std_msgs::Header& header);

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
  ros::Publisher map3D_features_publisher_;
  ros::Publisher map3D_corners_publisher_;
  ros::Publisher map3D_planars_publisher_;
  ros::Publisher map3D_planes_publisher_;
  ros::Publisher pose_publisher_;
  ros::Publisher odom_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher poses_publisher_;
  ros::Publisher gps_path_publisher_;
  ros::Publisher gps_pose_publisher_;
  ros::Publisher corners_local_publisher_;
  ros::Publisher planars_local_publisher_;
  ros::Publisher planes_local_publisher_;
  ros::ServiceClient polar2pose_;
  ros::ServiceClient set_datum_;

  // Classes object members
  Parameters params_;
  Localizer* localizer_;
  ElevationMap* elevation_map_;
  OccupancyMap* grid_map_;
  OccupancyMap* previous_map_;
  LandmarkMapper* land_mapper_;
  VisualMapper* vis_mapper_;
  LidarMapper* lid_mapper_;
  Observation obsv_;
  Timer timer_;

  // Array of poses to store and publish the robot path
  std::vector<geometry_msgs::PoseStamped> path_;
  std::vector<geometry_msgs::PoseStamped> gps_poses_;

  // Motion variables
  Pose init_odom_pose_;
  Pose robot_pose_;

  // GNSS variables
  int datum_autocorrection_stage_;
  int32_t global_counter_;
  float datum_orientation_[360][4]{};
  bool has_converged_{};
  bool estimate_heading_;
  float heading_;

  // Initialization flags
  bool init_flag_;
  bool init_gps_;
  bool init_odom_;
  bool register_map_;
};

}  // namespace vineslam
