#pragma once

// std & yaml & eigen
#include <iostream>
#include <deque>
#include <eigen3/Eigen/Dense>

#include "../params.hpp"
#include "../feature/three_dimensional.hpp"
#include "../mapping/occupancy_map.hpp"
#include "../math/point.hpp"
#include "../math/pose.hpp"
#include "../math/tf.hpp"
#include "../math/const.hpp"

namespace vineslam
{

// --------------------------------------------------------------------------------
// -- UTILS FOR CLOUD SEGMENTATION AND FEATURE EXTRACTION
struct SegPCL {
  std::vector<bool>  is_ground;
  std::vector<int>   start_col_idx;
  std::vector<int>   end_col_idx;
  std::vector<int>   col_idx;
  std::vector<float> range;
};

struct smoothness_t {
  float  value;
  size_t idx;
};

struct by_value {
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

  void registerMaps(const pose&          robot_pose,
                    std::vector<Corner>& corners,
                    std::vector<Planar>& planars,
                    std::vector<Plane>&  planes,
                    OccupancyMap&        grid_map);

  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Builds local map given the current 3D point cloud - for velodyne
  void localMap(const std::vector<point>& pcl,
                std::vector<Corner>&      out_corners,
                std::vector<Planar>&      out_planars,
                std::vector<Plane>&       out_planes,
                Plane&                    out_groundplane);
  void setVel2Base(const float& x,
                   const float& y,
                   const float& z,
                   const float& roll,
                   const float& pitch,
                   const float& yaw)
  {
    vel2base_x     = x;
    vel2base_y     = y;
    vel2base_z     = z;
    vel2base_roll  = roll;
    vel2base_pitch = pitch;
    vel2base_yaw   = yaw;
  }

  // Public vars
  float lidar_height;

private:
  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Adds the corner features to the global map
  void globalCornerMap(const pose&          robot_pose,
                       std::vector<Corner>& corners,
                       OccupancyMap&        grid_map) const;
  // Adds the planar features to the global map
  void globalPlanarMap(const pose&          robot_pose,
                       std::vector<Planar>& planars,
                       OccupancyMap&        grid_map) const;

  // Method to reset all the global variables and members
  void reset();

  // Method that extracts the ground plane of an input point cloud
  void groundRemoval(const std::vector<point>& in_pts, Plane& out_pcl);

  // Cloud generic plane segmentation
  void cloudSegmentation(const std::vector<point>& in_pts,
                         std::vector<PlanePoint>&  cloud_seg,
                         std::vector<PlanePoint>&  cloud_seg_pure);

  // Label a segment of a 3D point cloud
  void labelComponents(const int&                row,
                       const int&                col,
                       const std::vector<point>& in_pts,
                       int&                      label);

  // Extract the couple of vegetation side planes
  static void extractHighLevelPlanes(const std::vector<PlanePoint>& in_plane_pts,
                                     std::vector<Plane>&            out_planes);

  // 3D feature extraction from a point cloud
  void extractFeatures(const std::vector<PlanePoint>& in_plane_pts,
                       std::vector<Corner>&           out_corners,
                       std::vector<Planar>&           out_planars);

  // Local occupancy grid map
  OccupancyMap* local_map;

  // 3D cloud feature parameters
  int   max_iters;
  float dist_threshold;
  float planes_th{};
  float ground_th{};
  float edge_threshold{};
  float planar_threshold{};
  // ----------------------------
  int   picked_num{};
  int   vertical_scans{};
  int   horizontal_scans{};
  int   ground_scan_idx{};
  int   segment_valid_point_num{};
  int   segment_valid_line_num{};
  float vertical_angle_bottom{};
  float ang_res_x{};
  float ang_res_y{};

  // Transformation parameters
  float vel2base_x{}, vel2base_y{}, vel2base_z{}, vel2base_roll{}, vel2base_pitch{},
      vel2base_yaw{};

  // Cloud segmentation matrices
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> range_mat;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>   ground_mat;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>   label_mat;
  // Cloud segmentation & feature extraction structure
  SegPCL seg_pcl;
};

} // namespace vineslam