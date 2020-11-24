#pragma once

// std & yaml & eigen
#include <iostream>

// Objects
#include "../params.hpp"
#include "../feature/visual.hpp"
#include "../mapping/occupancy_map.hpp"
#include "../math/Point.hpp"
#include "../math/Pose.hpp"
#include "../math/Tf.hpp"
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

  void registerMaps(const Pose& robot_pose, std::vector<ImageFeature>& img_features, OccupancyMap& grid_map);

  // -------------------------------------------------------------------------------
  // ---- 3D image feature map
  // -------------------------------------------------------------------------------
  // Builds local map given the current image feature observations
  void localMap(const cv::Mat& img, const float* depths, std::vector<ImageFeature>& out_features);

  // Setter functions
  void setCam2Base(const float& x, const float& y, const float& z, const float& roll, const float& pitch,
                   const float& yaw)
  {
    cam2base_x_ = x;
    cam2base_y_ = y;
    cam2base_z_ = z;
    cam2base_roll_ = roll;
    cam2base_pitch_ = pitch;
    cam2base_yaw_ = yaw;
  }

private:
  // -------------------------------------------------------------------------------
  // ---- 3D image feature map
  // -------------------------------------------------------------------------------
  // Adds the image features to the global map
  void globalMap(const std::vector<ImageFeature>& features, const Pose& robot_pose, OccupancyMap& grid_map) const;

  // Perform feature extraction
  void extractSurfFeatures(const cv::Mat& in, std::vector<ImageFeature>& out) const;
  // Converts a pixel into world's coordinate reference
  void pixel2base(const Point& in_pt, const float& depth, Point& out_pt) const;

  // Camera info parameters
  float fx_;
  float fy_;
  float cx_;
  float cy_;

  // 3D map parameters
  float max_range_;
  float max_height_;
  int hessian_threshold_;

  // Transformation parameters
  float cam2base_x_{}, cam2base_y_{}, cam2base_z_{}, cam2base_roll_{}, cam2base_pitch_{}, cam2base_yaw_{};
};

}  // namespace vineslam
