#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vineslam/params.hpp>

namespace vineslam
{
static void loadParameters(const rclcpp::Node& nh, const std::string& prefix, Parameters& params)
{
  const std::string& node_name = nh.get_name();

  // Load params
  if (!nh.get_parameter(prefix + "/bagfile_name", params.bagfile_name_) && node_name == "/replay_node")
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/bagfile_name parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/odom_topic", params.odom_topic_))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/odom_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/tf_topic", params.tf_topic_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/tf_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/fix_topic", params.fix_topic_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/fix_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/image_features_topic", params.image_features_topic_) && !(node_name == "/mapping_"
                                                                                                          "node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/img_features_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/detections_topic", params.detections_topic_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/detections_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pcl_topic", params.pcl_topic_))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/pcl_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/system/use_semantic_features", params.use_semantic_features_) && !(node_name == "map"
                                                                                                                   "pin"
                                                                                                                   "g_"
                                                                                                                   "nod"
                                                                                                                   "e"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/system/semantic_features parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/system/use_lidar_features", params.use_lidar_features_) && !(node_name == "/mapping_"
                                                                                                             "node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/system/use_lidar_features parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/system/use_image_features", params.use_image_features_) && !(node_name == "/mapping_"
                                                                                                             "node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/system/use_image_features parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/system/use_gps", params.use_gps_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/system/use_gps parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/system/use_wheel_odometry", params.use_wheel_odometry_) && !(node_name == "/mapping_"
                                                                                                             "node"))
  {
    RCLCPP_WARN(nh.get_logger(), "%s/use_wheel_odometry parameter has not been set. Not using it...", prefix.c_str());
  }
  if (!nh.get_parameter(prefix + "/system/gps_datum/lat", params.latitude_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/system/gps_datum/lat parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/system/gps_datum/long", params.longitude_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/system/gps_datum/long parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }

  if (!nh.get_parameter(prefix + "/camera_info/baseline", params.baseline_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/camera_info/baseline parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/camera_info/fx", params.fx_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/camera_info/fx parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/camera_info/cx", params.cx_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(), "%s/camera_info/cx parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/ICP/distance_threshold", params.icp_distance_threshold_) &&
      !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/ICP/distance_threshold parameter not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/ICP/max_iters", params.icp_max_iters_) && !(node_name == "/mappin"
                                                                                                               "g_"
                                                                                                               "node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/ICP/max_iters parameter not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/ICP/reject_outliers", params.icp_reject_outliers_) &&
      !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/ICP/max_iters parameter not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/grid_map/origin/x", params.gridmap_origin_x_) &&
      !(node_name == "/loca"
                     "lizat"
                     "ion_"
                     "nod"
                     "e"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/grid_map/origin/x parameter "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/grid_map/origin/y", params.gridmap_origin_y_) &&
      !(node_name == "/loca"
                     "lizat"
                     "ion_"
                     "nod"
                     "e"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/grid_map/origin/y parameter "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/grid_map/origin/z", params.gridmap_origin_z_) &&
      !(node_name == "/loca"
                     "lizat"
                     "ion_"
                     "nod"
                     "e"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/grid_map/origin/z parameter "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/grid_map/width", params.gridmap_width_) && !(node_name == "/local"
                                                                                                                "izati"
                                                                                                                "on_"
                                                                                                                "node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/grid_map/width "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/grid_map/lenght", params.gridmap_lenght_) && !(node_name == "/loc"
                                                                                                                  "aliz"
                                                                                                                  "a"
                                                                                                                  "tion"
                                                                                                                  "_"
                                                                                                                  "nod"
                                                                                                                  "e"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/grid_map/lenght "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/grid_map/height", params.gridmap_height_) && !(node_name == "/loc"
                                                                                                                  "aliz"
                                                                                                                  "a"
                                                                                                                  "tion"
                                                                                                                  "_"
                                                                                                                  "nod"
                                                                                                                  "e"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/grid_map/height "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/grid_map/resolution", params.gridmap_resolution_) &&
      !(node_name == "/localization_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/grid_map/resolution "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/grid_map/save_map", params.save_map_) && !(node_name == "/localiz"
                                                                                                              "ation"
                                                                                                              "_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/grid_map/save_map "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/grid_map/output_folder", params.map_output_folder_) &&
      !(node_name == "/localization_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/grid_map/output_folder "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/multilayer_mapping/grid_map/input_file", params.map_input_file_) &&
      !(node_name == "/mapp"
                     "ing_"
                     "nod"
                     "e"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/multilayer_mapping/grid_map/input_file "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/n_particles", params.number_particles_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/n_particles "
                 "not found. Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_xx", params.sigma_xx_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_xx not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_yy", params.sigma_yy_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_yy not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_zz", params.sigma_zz_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_zz not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_RR", params.sigma_RR_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_RR not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_PP", params.sigma_PP_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_pitch not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_YY", params.sigma_YY_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_YY not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_landmark_matching", params.sigma_landmark_matching_) && !(node_name == "/ma"
                                                                                                                   "ppi"
                                                                                                                   "ng"
                                                                                                                   "_no"
                                                                                                                   "d"
                                                                                                                   "e"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_landmark_matching not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_feature_matching", params.sigma_feature_matching_) && !(node_name == "/mapp"
                                                                                                                 "ing_"
                                                                                                                 "nod"
                                                                                                                 "e"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_feature_matching not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_corner_matching", params.sigma_corner_matching_) && !(node_name == "/mappin"
                                                                                                               "g_"
                                                                                                               "node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_corner_matching not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_planar_matching", params.sigma_planar_matching_) && !(node_name == "/mappin"
                                                                                                               "g_"
                                                                                                               "node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_planar_matching not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/sigma_gps", params.sigma_gps_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/sigma_gps not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }
  if (!nh.get_parameter(prefix + "/pf/k_clusters", params.number_clusters_) && !(node_name == "/mapping_node"))
  {
    RCLCPP_ERROR(nh.get_logger(),
                 "%s/pf/k_clusters not found. "
                 "Shutting down...",
                 prefix.c_str());
    exit(-1);
  }

  rclcpp::Parameter cam2base_param = nh.get_parameter(prefix + "/system/cam2base");
  params.cam2base_ = cam2base_param.as_double_array();
  rclcpp::Parameter vel2base_param = nh.get_parameter(prefix + "/system/vel2base");
  params.cam2base_ = vel2base_param.as_double_array();
}

}  // namespace vineslam