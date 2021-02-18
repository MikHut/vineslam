#pragma once

#include <iostream>
#include <vector>

namespace vineslam
{
struct Parameters
{
  // -----------------------------------
  // ------ ROS topic names
  // -----------------------------------
  std::string bagfile_name_;
  std::string odom_topic_;
  std::string tf_topic_;
  std::string fix_topic_;
  std::string image_features_topic_;
  std::string detections_topic_;
  std::string pcl_topic_;

  // -----------------------------------
  // ------ System flags
  // -----------------------------------
  bool use_semantic_features_{};
  bool use_lidar_features_{};
  bool use_image_features_{};
  bool use_gps_{};
  bool use_wheel_odometry_{};

  // -----------------------------------
  // ------ GPS datum
  // -----------------------------------
  float latitude_{};
  float longitude_{};

  // -----------------------------------
  // ------ On-board sensors transformations to base link
  // -----------------------------------
  std::vector<double> cam2base_;
  std::vector<double> vel2base_;

  // -----------------------------------
  // ------ Camera info parameters
  // -----------------------------------
  float baseline_{};
  float fx_{};
  float cx_{};

  // -----------------------------------
  // ------ Multi-layer mapping parameters
  // -----------------------------------
  // - Grid map
  float gridmap_origin_x_{};
  float gridmap_origin_y_{};
  float gridmap_origin_z_{};
  float gridmap_width_{};
  float gridmap_lenght_{};
  float gridmap_height_{};
  float gridmap_resolution_{};
  bool save_map_{};
  std::string map_output_folder_;
  std::string map_input_file_;
  // - 3D map
  int icp_max_iters_{};
  float icp_distance_threshold_{};
  bool icp_reject_outliers_{};

  // -----------------------------------
  // ------ Particle filter parameters
  // -----------------------------------
  int number_particles_{};
  float sigma_xx_{};
  float sigma_yy_{};
  float sigma_zz_{};
  float sigma_RR_{};
  float sigma_PP_{};
  float sigma_YY_{};
  float sigma_landmark_matching_{};
  float sigma_feature_matching_{};
  float sigma_corner_matching_{};
  float sigma_planar_matching_{};
  float sigma_gps_{};
  int number_clusters_{};

  // -----------------------------------
  // ------ METHODS
  // -----------------------------------
  Parameters() = default;
};

}  // namespace vineslam