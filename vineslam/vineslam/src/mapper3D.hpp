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
                     OccupancyMap&                    grid_map);

  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Builds local map given the current 3D point cloud
  void localPCLMap(const float*         depths,
                   std::vector<Corner>& out_corners,
                   Plane&               out_groundplane);

  // Adds the corner features to the global map
  void globalCornerMap(const std::vector<Corner>& corners,
                       const pose&                robot_pose,
                       OccupancyMap&              grid_map);

private:
  // -------------------------------------------------------------------------------
  // ---- 3D image feature map
  // -------------------------------------------------------------------------------
  // Perform feature extraction
  void extractSurfFeatures(const cv::Mat& in, std::vector<ImageFeature>& out);

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

  // 3D feature extraction from a point cloud
  void extractCorners(const std::vector<PlanePoint>& in_plane_pts,
                      std::vector<Corner>&           out_corners);

  // ------------------------------------------------------------------------------

  // Converts a pixel into world's coordinate reference
  void pixel2world(const point& in_pt, const float& depth, point& out_pt) const;

  // Camera info parameters
  int   img_width;
  int   img_height;
  float cam_height;
  float cam_pitch;
  float angle_hres;
  float angle_vres;
  float fx;
  float fy;
  float cx;
  float cy;
  // 3D map parameters
  float       max_range;
  float       max_height;
  std::string fdetector;
  // 3D cloud feature parameters
  int   downsample_f;
  int   max_iters;
  float dist_threshold;
  float planes_th;
  float ground_th;
  float edge_threshold;

  // Cloud segmentation matrices
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> range_mat;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>   ground_mat;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>   label_mat;
  // Cloud segmentation & feature extraction structure
  SegPCL seg_pcl;
};
}; // namespace vineslam
