#pragma once

#include <iostream>
#include <vector>

namespace vineslam
{
struct Parameters
{
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


  // -----------------------------------
  // ------ METHODS
  // -----------------------------------
  Parameters() = default;
};

}  // namespace vineslam