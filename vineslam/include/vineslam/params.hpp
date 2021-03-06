#pragma once

#include <iostream>
#include <vector>

namespace vineslam
{
struct Parameters
{
  // -----------------------------------
  // ------ System settings
  // -----------------------------------
  std::string world_frame_id_{};
  std::string lidar_sensor_frame_{};
  std::string camera_sensor_frame_{};

  // -----------------------------------
  // ------ System flags
  // -----------------------------------
  bool use_semantic_features_{};
  bool use_lidar_features_{};
  bool use_vertical_planes_{};
  bool use_gps_{};
  bool use_gps_altitude_{};
  bool use_imu_{};
  bool use_gyroscope_{};
  bool register_maps_{};
  bool lightweight_version_{};

  // -----------------------------------
  // ------ Logs
  // -----------------------------------
  bool save_logs_{};
  std::string logs_folder_{};

  // -----------------------------------
  // ------ Map origin - datum
  // -----------------------------------
  double map_datum_lat_{};
  double map_datum_long_{};
  double map_datum_alt_{};
  double map_datum_head_{};
  double robot_datum_lat_{};
  double robot_datum_long_{};
  double robot_datum_alt_{};
  double robot_datum_head_{};
  double sat_datum_lat_{};
  double sat_datum_long_{};

  // -----------------------------------
  // ------ Camera info parameters
  // -----------------------------------
  float baseline_{};
  float fx_{};
  float cx_{};
  float h_fov_{};
  float v_fov_{};
  int img_width_{};
  int img_height_{};

  // -----------------------------------
  // ------ Robot dimensions
  // -----------------------------------
  float robot_dim_x_;
  float robot_dim_y_;
  float robot_dim_z_;

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
  std::string map_output_folder_;
  std::string map_input_file_;
  std::string elevation_map_input_file_;
  std::string topological_map_input_file_;

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