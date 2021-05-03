#pragma once

// std & yaml & eigen
#include <iostream>
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
// --------------------------------------------------------------------------------
// -- UTILS FOR CLOUD SEGMENTATION AND FEATURE EXTRACTION
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

class LidarMapper
{
public:
  // Class constructor - receives and saves the system
  // parameters
  explicit LidarMapper(const Parameters& params);

  void registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners, const std::vector<Planar>& planars,
                    const std::vector<SemiPlane>& planes, const SemiPlane& ground, OccupancyMap& grid_map,
                    ElevationMap& elevation_map);
  void registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners, const std::vector<Planar>& planars,
                    const std::vector<SemiPlane>& planes, const SemiPlane& ground, OccupancyMap& grid_map);

  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Builds local map given the current 3D point cloud - for velodyne
  void localMap(const std::vector<Point>& pcl, std::vector<Corner>& out_corners, std::vector<Planar>& out_planars,
                std::vector<SemiPlane>& out_planes, SemiPlane& out_groundplane);
  void setVel2Base(const float& x, const float& y, const float& z, const float& roll, const float& pitch,
                   const float& yaw)
  {
    vel2base_x_ = x;
    vel2base_y_ = y;
    vel2base_z_ = z;
    vel2base_roll_ = roll;
    vel2base_pitch_ = pitch;
    vel2base_yaw_ = yaw;
  }

  // Routine to compute the unoccupied zone around the robot used to refine the previously built map
  void computeUnoccupiedZone(const std::vector<Point>& in_pts, std::vector<Point>& rectangle);
  // Routine to filter the map using the unoccupied computed zone
  void filterWithinZone(const Pose& robot_pose, const std::vector<Point>& rectangle, OccupancyMap& grid_map);

  // Public vars
  float lidar_height;

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

  // Previous robot pose
  Pose prev_robot_pose_;

  // Robot dimensions vars
  float robot_dim_x_;
  float robot_dim_y_;
  float robot_dim_z_;

  // 3D cloud feature parameters
  float planes_th_{};
  float ground_th_{};
  float edge_threshold_{};
  float planar_threshold_{};
  // ----------------------------
  int picked_num_{};
  int vertical_scans_{};
  int horizontal_scans_{};
  int ground_scan_idx_{};
  int segment_valid_point_num_{};
  int segment_valid_line_num_{};
  float vertical_angle_bottom_{};
  float ang_res_x_{};
  float ang_res_y_{};
  int filter_frequency_{};

  // Mapper iterator
  int it_{};

  // Transformation parameters
  float vel2base_x_{}, vel2base_y_{}, vel2base_z_{}, vel2base_roll_{}, vel2base_pitch_{}, vel2base_yaw_{};

  // Cloud segmentation matrices
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> range_mat_;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> ground_mat_;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> label_mat_;
  // Cloud segmentation & feature extraction structure
  SegPCL seg_pcl_;
};

}  // namespace vineslam