#pragma once

// std & yaml & eigen
#include <iostream>

// Objects
#include "../params.hpp"
#include "../feature/visual.hpp"
#include "../mapping/occupancy_map.hpp"
#include "../math/point.hpp"
#include "../math/pose.hpp"
#include "../math/tf.hpp"
#include "../math/const.hpp"

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

namespace vineslam
{

class VisualMapper
{
public:
  // Class constructor - receives and saves the system
  // parameters
  explicit VisualMapper(const Parameters& params);

  void registerMaps(const pose&                robot_pose,
                    std::vector<ImageFeature>& img_features,
                    OccupancyMap&              grid_map);

  // -------------------------------------------------------------------------------
  // ---- 3D image feature map
  // -------------------------------------------------------------------------------
  // Builds local map given the current image feature observations
  void localMap(const cv::Mat&             img,
                const float*               depths,
                std::vector<ImageFeature>& out_features);

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

private:
  // -------------------------------------------------------------------------------
  // ---- 3D image feature map
  // -------------------------------------------------------------------------------
  // Adds the image features to the global map
  void globalMap(const std::vector<ImageFeature>& features,
                 const pose&                      robot_pose,
                 OccupancyMap&                    grid_map) const;

  // Perform feature extraction
  void extractSurfFeatures(const cv::Mat& in, std::vector<ImageFeature>& out) const;
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
  float max_range;
  float max_height;
  int   hessian_threshold;

  // Transformation parameters
  float cam2base_x{}, cam2base_y{}, cam2base_z{}, cam2base_roll{}, cam2base_pitch{},
      cam2base_yaw{};
};

} // namespace vineslam
