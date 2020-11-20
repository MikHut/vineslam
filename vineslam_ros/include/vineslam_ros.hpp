#pragma once

// vineslam members
#include <params_loader.hpp>
#include <vineslam/feature/semantic.hpp>
#include <vineslam/feature/visual.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/localization/localizer.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/mapping/landmark_mapping.hpp>
#include <vineslam/mapping/visual_mapping.hpp>
#include <vineslam/mapping/lidar_mapping.hpp>
#include <vineslam/math/point.hpp>
#include <vineslam/math/pose.hpp>
#include <vineslam/math/const.hpp>
#include <vineslam/mapxml/map_writer.hpp>
#include <vineslam/mapxml/map_parser.hpp>
#include <vineslam/utils/save_data.hpp>
// ----------------------------
#include <vineslam_msgs/particle.h>
#include <vineslam_msgs/report.h>
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
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <vineslam_ros/start_map_registration.h>
#include <vineslam_ros/stop_map_registration.h>
#include <vineslam_ros/stop_gps_heading_estimation.h>

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
  void init();
  void loop();
  void loopOnce();
  void process();

  // Stereo camera images callback function
  void imageListener(const sensor_msgs::ImageConstPtr& rgb_image,
                     const sensor_msgs::ImageConstPtr& depth_image);
  void _imageListener(const cv::Mat&                    rgb_image,
                      const sensor_msgs::ImageConstPtr& depth_image);
  // Landmark detection callback function
  void landmarkListener(const vision_msgs::Detection2DArrayConstPtr& dets);
  // Scan callback function
  void scanListener(const sensor_msgs::PointCloud2ConstPtr& msg);
  // Odometry callback function
  void odomListener(const nav_msgs::OdometryConstPtr& msg);
  // GPS callback function
  void gpsListener(const sensor_msgs::NavSatFixConstPtr& msg);
  // Services callbacks
  bool startRegistration(vineslam_ros::start_map_registration::Request&,
                         vineslam_ros::start_map_registration::Response&);
  bool stopRegistration(vineslam_ros::stop_map_registration::Request&,
                        vineslam_ros::stop_map_registration::Response&);
  bool stopHeadingEstimation(vineslam_ros::stop_gps_heading_estimation::Request&,
                             vineslam_ros::stop_gps_heading_estimation::Response&);

  // Publish 2D semantic features map
  void publish2DMap(const pose&               pose,
                    const std::vector<float>& bearings,
                    const std::vector<float>& depths) const;
  // Publish the 3D maps
  void publish3DMap() const;
  // Publish the 3D PCL planes
  void publish3DMap(const std::vector<Plane>& planes, const ros::Publisher& pub);
  void publish3DMap(const pose&               r_pose,
                    const std::vector<Plane>& planes,
                    const ros::Publisher&     pub);
  // Publish a 3D PCL corners map
  void publish3DMap(const std::vector<Corner>& corners, const ros::Publisher& pub);
  void publish3DMap(const pose&                r_pose,
                    const std::vector<Corner>& corners,
                    const ros::Publisher&      pub);
  // Publish a 3D PCL planar features map
  void publish3DMap(const std::vector<Planar>& planars, const ros::Publisher& pub);
  void publish3DMap(const pose&                r_pose,
                    const std::vector<Planar>& planars,
                    const ros::Publisher&      pub);
  // Publish the grid map that contains all the maps
  void publishGridMap(const std_msgs::Header& header) const;

  // GNSS heading estimator
  bool getGNSSHeading(const pose& gps_odom, const std_msgs::Header& header);

  // VineSLAM input data
  struct InputData {
    // Landmark labels array
    std::vector<int> land_labels;
    // Landmark bearings array
    std::vector<float> land_bearings;
    // Landmark depths array
    std::vector<float> land_depths;
    // Depth image transformed into a linear vector
    float* depth_array;
    // Input RGB image
    cv::Mat rgb_image;
    // Wheel odometry pose
    pose wheel_odom_pose;
    // GNSS pose
    pose gnss_pose;
    // LiDAR scan points
    std::vector<point> scan_pts;

    // Observation flags
    bool received_landmarks;
    bool received_images;
    bool received_odometry;
    bool received_gnss;
    bool received_scans;
  } input_data;

  // ROS publishers/services
  ros::Publisher     vineslam_report_publisher;
  ros::Publisher     grid_map_publisher;
  ros::Publisher     map2D_publisher;
  ros::Publisher     map3D_features_publisher;
  ros::Publisher     map3D_corners_publisher;
  ros::Publisher     map3D_planars_publisher;
  ros::Publisher     pose_publisher;
  ros::Publisher     path_publisher;
  ros::Publisher     poses_publisher;
  ros::Publisher     gps_publisher;
  ros::Publisher     corners_local_publisher;
  ros::Publisher     planars_local_publisher;
  ros::Publisher     planes_local_publisher;
  ros::Publisher     debug_markers;
  ros::ServiceClient polar2pose;
  ros::ServiceClient set_datum;

  // Classes object members
  Parameters      params;
  Localizer*      localizer;
  OccupancyMap*   grid_map;
  OccupancyMap*   previous_map;
  LandmarkMapper* land_mapper;
  VisualMapper*   vis_mapper;
  LidarMapper*    lid_mapper;
  Observation     obsv;

  // Array of poses to store and publish the robot path
  std::vector<geometry_msgs::PoseStamped> path;
  std::vector<geometry_msgs::PoseStamped> gps_poses;

  // Motion variables
  pose init_odom_pose;
  pose robot_pose;

  // GNSS variables
  int     datum_autocorrection_stage;
  int32_t global_counter;
  float   datum_orientation[360][4]{};
  bool    has_converged{};
  bool    estimate_heading;
  float   heading;

  // Initialization flags
  bool init_flag;
  bool init_gps;
  bool init_odom;
  bool register_map;
};

} // namespace vineslam
