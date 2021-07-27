#pragma once

// std & yaml & eigen
#include <iostream>
#include <unordered_map>
#include <deque>
#include <eigen3/Eigen/Dense>

#include <vineslam/params.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/mapping/elevation_map.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Tf.hpp>
#include <vineslam/math/Const.hpp>
#include <vineslam/filters/ransac.hpp>
#include <vineslam/filters/convex_hull.hpp>
#include <vineslam/utils/Timer.hpp>

namespace vineslam
{
class LidarMapper
{
public:
  // Class constructor - receives and saves the system
  // parameters
  explicit LidarMapper();

  void registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners, const std::vector<Planar>& planars,
                    const std::vector<SemiPlane>& planes, const SemiPlane& ground, OccupancyMap& grid_map,
                    ElevationMap& elevation_map);
  void registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners, const std::vector<Planar>& planars,
                    const std::vector<SemiPlane>& planes, const SemiPlane& ground, OccupancyMap& grid_map);

  // Routine to compute the unoccupied zone around the robot used to refine the previously built map
  void computeUnoccupiedZone(const std::vector<Point>& in_pts, std::vector<Point>& rectangle);
  // Routine to filter the map using the unoccupied computed zone
  void filterWithinZone(const Pose& robot_pose, const std::vector<Point>& rectangle, OccupancyMap& grid_map);

  void setLaser2Base(const float& x, const float& y, const float& z, const float& roll, const float& pitch,
                     const float& yaw)
  {
    laser2base_x_ = x;
    laser2base_y_ = y;
    laser2base_z_ = z;
    laser2base_roll_ = roll;
    laser2base_pitch_ = pitch;
    laser2base_yaw_ = yaw;
  }

  // Public vars
  float lidar_height_;

  // Previous robot pose
  Pose prev_robot_pose_;

  // Robot dimensions vars
  float robot_dim_x_;
  float robot_dim_y_;
  float robot_dim_z_;

  // Mapper iterator
  int it_{};
  // Plane filter frequency
  int filter_frequency_{};

  // Transformation parameters
  float laser2base_x_{}, laser2base_y_{}, laser2base_z_{}, laser2base_roll_{}, laser2base_pitch_{}, laser2base_yaw_{};

private:
  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Adds the corner features to the global map
  static void globalCornerMap(const Pose& robot_pose, const std::vector<Corner>& corners, OccupancyMap& grid_map);
  // Adds the planar features to the global map
  static void globalPlanarMap(const Pose& robot_pose, const std::vector<Planar>& planars, OccupancyMap& grid_map);
  // Adds the plane features to the global map
  void globalPlaneMap(const Pose& robot_pose, const std::vector<SemiPlane>& planes, OccupancyMap& grid_map);
  // Adds new altemetry measures to the elevation map
  void globalElevationMap(const Pose& robot_pose, const Plane& ground_plane, ElevationMap& elevation_map);
};

// ----------------------------------------------
// ---------- Velodyne
// ----------------------------------------------

struct SegPCL
{
  std::vector<bool> is_ground;
  std::vector<int> start_col_idx;
  std::vector<int> end_col_idx;
  std::vector<int> col_idx;
  std::vector<float> range;
};

struct smoothness_t
{
  float value;
  size_t idx;
};

struct by_value
{
  bool operator()(smoothness_t const& left, smoothness_t const& right)
  {
    return left.value < right.value;
  }
};

class VelodyneMapper : public LidarMapper
{
public:
  VelodyneMapper(const Parameters& params);

  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Builds local map given the current 3D point cloud - for velodyne
  void localMap(const std::vector<Point>& pcl, std::vector<Corner>& out_corners, std::vector<Planar>& out_planars,
                std::vector<SemiPlane>& out_planes, SemiPlane& out_groundplane);
  void localMap(const std::vector<Point>& pcl, std::vector<Corner>& out_corners, std::vector<Planar>& out_planars,
                SemiPlane& out_groundplane);

private:
  // Method to reset all the global variables and members
  void reset();

  // Method that extracts the ground plane of an input point cloud
  void groundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl);
  void flatGroundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl);

  // Cloud generic plane segmentation
  void cloudSegmentation(const std::vector<Point>& in_pts, std::vector<PlanePoint>& cloud_seg);

  // Label a segment of a 3D point cloud
  void labelComponents(const int& row, const int& col, int& label);

  // Extract a couple of semiplanes
  void extractHighLevelPlanes(const std::vector<Point>& in_plane_pts, const SemiPlane& ground_plane,
                              std::vector<SemiPlane>& out_planes);
  bool checkPlaneConsistency(const SemiPlane& plane, const SemiPlane& ground_plane);

  // 3D feature extraction from a point cloud
  void extractFeatures(const std::vector<PlanePoint>& in_plane_pts, std::vector<Corner>& out_corners,
                       std::vector<Planar>& out_planars);

  // Cloud segmentation matrices
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> range_mat_;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> ground_mat_;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> label_mat_;
  // Cloud segmentation & feature extraction structure
  SegPCL seg_pcl_;

  // 3D cloud feature parameters
  float planes_th_{};
  float ground_th_{};
  float edge_threshold_{};
  float planar_threshold_{};
  int picked_num_{};
  int vertical_scans_{};
  int horizontal_scans_{};
  int ground_scan_idx_{};
  int segment_valid_point_num_{};
  int segment_valid_line_num_{};
  float vertical_angle_bottom_{};
  float ang_res_x_{};
  float ang_res_y_{};
};

// ----------------------------------------------
// ---------- Livox
// ----------------------------------------------

enum E_point_type
{
  e_pt_normal = 0,                       // normal points
  e_pt_000 = 0x0001 << 0,                // points [0,0,0]
  e_pt_too_near = 0x0001 << 1,           // points in short range
  e_pt_reflectivity_low = 0x0001 << 2,   // low reflectivity
  e_pt_reflectivity_high = 0x0001 << 3,  // high reflectivity
  e_pt_circle_edge = 0x0001 << 4,        // points near the edge of circle
  e_pt_nan = 0x0001 << 5,                // points with infinite value
  e_pt_small_view_angle = 0x0001 << 6,   // points with large viewed angle
};

enum E_feature_type  // if and only if normal point can be labeled
{
  e_label_invalid = -1,
  e_label_unlabeled = 0,
  e_label_corner = 0x0001 << 0,
  e_label_surface = 0x0001 << 1,
  e_label_near_nan = 0x0001 << 2,
  e_label_near_zero = 0x0001 << 3,
  e_label_hight_intensity = 0x0001 << 4,
};

// Encode point infos using points intensity, which is more convenient for debugging.
enum E_intensity_type
{
  e_I_raw = 0,
  e_I_motion_blur,
  e_I_motion_mix,
  e_I_sigma,
  e_I_scan_angle,
  e_I_curvature,
  e_I_view_angle,
  e_I_time_stamp
};

struct Pt_infos
{
  int pt_type_ = e_pt_normal;
  int pt_label_ = e_label_unlabeled;
  int idx_ = 0.f;
  float raw_intensity_ = 0.f;
  float time_stamp_ = 0.0;
  float polar_angle_ = 0.f;
  int polar_direction_ = 0;
  float polar_dis_sq2_ = 0.f;
  float depth_sq2_ = 0.f;
  float curvature_ = 0.0;
  float view_angle_ = 0.0;
  float sigma_ = 0.0;
  Eigen::Matrix<float, 2, 1> pt_2d_img_;  // project to X==1 plane
};

struct Pt_compare
{
  inline bool operator()(const Point& a, const Point& b)
  {
    return ((a.x_ < b.x_) || (a.x_ == b.x_ && a.y_ < b.y_) || ((a.x_ == b.x_) && (a.y_ == b.y_) && (a.z_ < b.z_)));
  }

  bool operator()(const Point& a, const Point& b) const
  {
    return (a.x_ == b.x_) && (a.y_ == b.y_) && (a.z_ == b.z_);
  }
};

struct Pt_hasher
{
  std::size_t operator()(const Point& k) const
  {
    return ((std::hash<float>()(k.x_) ^ (std::hash<float>()(k.y_) << 1)) >> 1) ^ (std::hash<float>()(k.z_) << 1);
  }
};

class LivoxMapper : public LidarMapper
{
public:
  LivoxMapper(const Parameters& params);

  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Builds local map given the current 3D point cloud - for velodyne
  void localMap(const std::vector<Point>& pcl, const double& time_stamp, std::vector<Corner>& out_corners,
                std::vector<Planar>& out_planars, std::vector<SemiPlane>& out_planes, SemiPlane& out_groundplane);
  void localMap(const std::vector<Point>& pcl, const double& time_stamp, std::vector<Corner>& out_corners,
                std::vector<Planar>& out_planars, SemiPlane& out_groundplane);

private:
  Pt_infos* findPtInfo(const Point& pt);
  void getFeatures(std::vector<Point>& pc_corners, std::vector<Point>& pc_surface, std::vector<Point>& pc_full_res,
                   float minimum_blur = 0.0, float maximum_blur = 0.3);
  void setIntensity(Point& pt, const E_intensity_type& i_type = e_I_motion_blur);
  std::vector<std::vector<Point>> extractLaserFeatures(const std::vector<Point>& laser_cloud_in, double time_stamp);
  void addMaskOfPoint(Pt_infos* pt_infos, const E_point_type& pt_type, int neighbor_count = 0);
  void evalPoint(Pt_infos* pt_info);
  int projectionScan3d2d(const std::vector<Point>& laser_cloud_in, std::vector<float>& scan_id_index);
  void splitLaserScan(const int clutter_size, const std::vector<Point>& laser_cloud_in,
                      const std::vector<float>& scan_id_index, std::vector<std::vector<Point>>& laser_cloud_scans);
  void computeFeatures();

  template <typename T>
  T vector_angle(const Eigen::Matrix<T, 3, 1>& vec_a, const Eigen::Matrix<T, 3, 1>& vec_b,
                 int if_force_sharp_angle = 0);

  // Method that extracts the ground plane of an input point cloud
  void flatGroundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl);
  // Extract a couple of semiplanes
  void extractHighLevelPlanes(const std::vector<Point>& in_plane_pts, const SemiPlane& ground_plane,
                              std::vector<SemiPlane>& out_planes);
  bool checkPlaneConsistency(const SemiPlane& plane, const SemiPlane& ground_plane);

  E_intensity_type default_return_intensity_type_;

  int pcl_data_save_index_;
  float max_fov_;  // Edge of circle to main axis
  float max_edge_polar_pos_;
  float time_internal_pts_;  // 10us = 1e-5
  float cx_;
  float cy_;
  int if_save_pcd_file_;
  int input_points_size_;
  double first_receive_time_;
  double current_time_;
  double last_maximum_time_stamp_;
  float thr_corner_curvature_;
  float thr_surface_curvature_;
  float minimum_view_angle_;
  std::vector<Pt_infos> pts_info_vec_;
  std::vector<Point> raw_pts_vec_;

  std::unordered_map<Point, Pt_infos*, Pt_hasher, Pt_compare> map_pt_idx_;  // using hash_map
  std::unordered_map<Point, Pt_infos*, Pt_hasher, Pt_compare>::iterator map_pt_idx_it_;

  float livox_min_allow_dis_;
  float livox_min_sigma_;

  float lidar_height_;
  int horizontal_scans_{};
  int vertical_scans_{};
  int ground_scan_idx_{};
  float ground_th_{};
  float vertical_angle_bottom_;
  float ang_res_x_;
  float ang_res_y_;
};

}  // namespace vineslam