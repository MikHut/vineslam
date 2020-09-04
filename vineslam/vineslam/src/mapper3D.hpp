#pragma once

// std & yaml & eigen
#include <iostream>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>

// Objects
#include <occupancy_map.hpp>
#include <math/point.hpp>
#include <math/pose.hpp>
#include <math/const.hpp>
#include <math/tf.hpp>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <feature.hpp>

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

// ---------------------------------------------------------------------------------
// The 3D map is composed of two layers:
// -> A map extracted directly from the 3D point cloud containing corners and the
// ground plane
// -> A map extracted from 2D image features and projected in 3D
// ---------------------------------------------------------------------------------
class Mapper3D
{
public:
  // Class constructor - receives and saves the system
  // parameters
  explicit Mapper3D(const std::string& config_path);

  // -------------------------------------------------------------------------------
  // ---- 3D image feature map
  // -------------------------------------------------------------------------------
  // Builds local map given the current image feature observations
  void localSurfMap(const cv::Mat&             img,
                    const float*               depths,
                    std::vector<ImageFeature>& out_features);

  // Adds the image features to the global map
  void globalSurfMap(const std::vector<ImageFeature>& features,
                     const pose&                      robot_pose,
                     OccupancyMap&                    grid_map) const;

  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Builds local map given the current 3D point cloud - for velodyne
  void localPCLMap(const std::vector<point>& pcl,
                   std::vector<Corner>&      out_corners,
                   std::vector<PlanePoint>&  out_cloud_seg,
                   Plane&                    out_groundplane);
  // Builds local map given the current 3D point cloud - for ZED
  void localPCLMap(const float*             depths,
                   std::vector<Corner>&     out_corners,
                   std::vector<PlanePoint>& out_cloud_seg,
                   Plane&                   out_groundplane);

  // Adds the corner features to the global map
  void globalCornerMap(const pose&          robot_pose,
                       std::vector<Corner>& corners,
                       OccupancyMap&        grid_map) const;
  // -------------------------------------------------------------------------------

  // Setter functions
  void setCam2Base(const float& x,
                   const float& y,
                   const float& z,
                   const float& roll,
                   const float& pitch,
                   const float& yaw)
  {
    cam2base_x     = x;
    cam2base_y     = y;
    cam2base_z     = z;
    cam2base_roll  = roll;
    cam2base_pitch = pitch;
    cam2base_yaw   = yaw;
  }
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

private:
  // -------------------------------------------------------------------------------
  // ---- 3D image feature map
  // -------------------------------------------------------------------------------
  // Perform feature extraction
  void extractSurfFeatures(const cv::Mat& in, std::vector<ImageFeature>& out) const;

  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Method to reset all the global variables and members
  void reset();

  // Method that extracts the ground plane of an input point cloud
  void groundRemoval(const std::vector<point>& in_pts, Plane& out_pcl);

  // Fit a 3D plane on a set of points using ransac
  bool ransac(const Plane& in_plane, Plane& out_plane) const;

  // Cloud generic plane segmentation
  void cloudSegmentation(const std::vector<point>& in_pts,
                         std::vector<PlanePoint>&  out_plane_pts);

  // Label a segment of a 3D point cloud
  void labelComponents(const int&                row,
                       const int&                col,
                       const std::vector<point>& in_pts,
                       int&                      label);

  // Extract the couple of vegetation side planes
  void extractVegetationPlanes(const std::vector<PlanePoint>& in_plane_pts,
                               std::vector<PlanePoint>&       out_planes);

  // 3D feature extraction from a point cloud
  void extractCorners(const std::vector<PlanePoint>& in_plane_pts,
                      std::vector<Corner>&           out_corners);

  // ------------------------------------------------------------------------------

  // Converts a pixel into world's coordinate reference
  void pixel2base(const point& in_pt, const float& depth, point& out_pt) const;

  // Camera info parameters
  int   img_width;
  int   img_height;
  float fx;
  float fy;
  float cx;
  float cy;
  float depth_hfov;
  float depth_vfov;

  // 3D map parameters
  std::string sensor;
  std::string metric;
  float       max_range;
  float       max_height;
  int         hessian_threshold;

  // 3D cloud feature parameters
  float correspondence_threshold;
  int   downsample_f;
  int   init_downsample_f;
  int   max_iters;
  float dist_threshold;
  float planes_th{};
  float ground_th{};
  float edge_threshold{};
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
  float cam2base_x{}, cam2base_y{}, cam2base_z{}, cam2base_roll{}, cam2base_pitch{},
      cam2base_yaw{};
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
