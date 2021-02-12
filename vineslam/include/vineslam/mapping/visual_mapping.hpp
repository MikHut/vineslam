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
  void localMap(const std::vector<ImageFeature>& in_features, std::vector<ImageFeature>& out_features);

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
  // Converts a pixel into world's coordinate reference
  void pixel2base(const Point& in_pt, Point& out_pt) const;

  // Transformation parameters
  float cam2base_x_{}, cam2base_y_{}, cam2base_z_{}, cam2base_roll_{}, cam2base_pitch_{}, cam2base_yaw_{};
};

}  // namespace vineslam
