#pragma once

#include <ros/ros.h>

#include <vineslam/params.hpp>

namespace vineslam
{
static void loadParameters(const ros::NodeHandle& nh, const std::string& prefix, Parameters& params)
{
  const std::string& node_name = ros::this_node::getName();

  // Load params
  if (!nh.getParam(prefix + "/bagfile_name", params.bagfile_name_) && node_name == "/replay_node")
  {
    ROS_ERROR("%s/bagfile_name parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/odom_topic", params.odom_topic_))
  {
    ROS_ERROR("%s/odom_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/tf_topic", params.tf_topic_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/tf_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/fix_topic", params.fix_topic_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/fix_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/depth_img_topic", params.depth_img_topic_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/depth_img_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/left_img_topic", params.rgb_img_topic_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/left_img_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/detections_topic", params.detections_topic_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/detections_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pcl_topic", params.pcl_topic_))
  {
    ROS_ERROR("%s/pcl_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_semantic_features", params.use_semantic_features_) && !(node_name == "/mapping"
                                                                                                              "_node"))
  {
    ROS_ERROR("%s/system/semantic_features parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_lidar_features", params.use_lidar_features_) && !(node_name == "/mapping_"
                                                                                                        "node"))
  {
    ROS_ERROR("%s/system/use_lidar_features parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_image_features", params.use_image_features_) && !(node_name == "/mapping_"
                                                                                                        "node"))
  {
    ROS_ERROR("%s/system/use_image_features parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_gps", params.use_gps_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/system/use_gps parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_wheel_odometry", params.use_wheel_odometry_) && !(node_name == "/mapping_"
                                                                                                        "node"))
  {
    ROS_WARN("%s/use_wheel_odometry parameter has not been set. Not using it...", prefix.c_str());
  }
  if (!nh.getParam(prefix + "/system/gps_datum/lat", params.latitude_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/system/gps_datum/lat parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/gps_datum/long", params.longitude_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/system/gps_datum/long parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/cam2base", params.cam2base_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/system/cam2base parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/vel2base", params.vel2base_))
  {
    ROS_ERROR("%s/system/vel2base parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/baseline", params.baseline_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/camera_info/baseline parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/depth_hfov", params.depth_hfov_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/camera_info/depth_hfov parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/depth_vfov", params.depth_vfov_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/camera_info/depth_vfov parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/cx", params.cx_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/camera_info/cx parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/cy", params.cy_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/camera_info/cy parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/fx", params.fx_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/camera_info/fx parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/fy", params.fy_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/camera_info/fy parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/img_width", params.img_width_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/camera_info/img_width parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/img_height", params.img_height_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/camera_info/img_height parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/ICP/distance_threshold", params.icp_distance_threshold_) &&
      !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/multilayer_mapping/ICP/distance_threshold parameter not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/ICP/max_iters", params.icp_max_iters_) && !(node_name == "/mapping_"
                                                                                                          "node"))
  {
    ROS_ERROR("%s/multilayer_mapping/ICP/max_iters parameter not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/ICP/reject_outliers", params.icp_reject_outliers_) &&
      !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/multilayer_mapping/ICP/max_iters parameter not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/origin/x", params.gridmap_origin_x_) && !(node_name == "/loca"
                                                                                                                 "lizat"
                                                                                                                 "ion_"
                                                                                                                 "nod"
                                                                                                                 "e"))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/origin/x parameter "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/origin/y", params.gridmap_origin_y_) && !(node_name == "/loca"
                                                                                                                 "lizat"
                                                                                                                 "ion_"
                                                                                                                 "nod"
                                                                                                                 "e"))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/origin/y parameter "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/origin/z", params.gridmap_origin_z_) && !(node_name == "/loca"
                                                                                                                 "lizat"
                                                                                                                 "ion_"
                                                                                                                 "nod"
                                                                                                                 "e"))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/origin/z parameter "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/width", params.gridmap_width_) && !(node_name == "/localizati"
                                                                                                           "on_node"))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/width "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/lenght", params.gridmap_lenght_) && !(node_name == "/localiza"
                                                                                                             "tion_"
                                                                                                             "node"))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/lenght "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/height", params.gridmap_height_) && !(node_name == "/localiza"
                                                                                                             "tion_"
                                                                                                             "node"))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/height "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/resolution", params.gridmap_resolution_) &&
      !(node_name == "/localization_node"))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/resolution "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/save_map", params.save_map_) && !(node_name == "/localization"
                                                                                                         "_node"))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/save_map "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/output_folder", params.map_output_folder_) &&
      !(node_name == "/localization_node"))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/output_folder "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/input_file", params.map_input_file_) && !(node_name == "/mapp"
                                                                                                                 "ing_"
                                                                                                                 "nod"
                                                                                                                 "e"))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/input_file "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/image_feature/hessian_threshold", params.hessian_threshold_) &&
      !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/multilayer_mapping/image_feature/hessian_threshold "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/map_3D/max_range", params.max_range_) && !(node_name == "/mapping_"
                                                                                                         "node"))
  {
    ROS_ERROR("%s/multilayer_mapping/map_3D/max_range "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/map_3D/max_height", params.max_height_) && !(node_name == "/mapping_"
                                                                                                           "node"))
  {
    ROS_ERROR("%s/multilayer_mapping/map_3D/max_height "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/n_particles", params.number_particles_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/pf/n_particles "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_xx", params.sigma_xx_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/pf/sigma_xx not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_yy", params.sigma_yy_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/pf/sigma_yy not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_zz", params.sigma_zz_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/pf/sigma_zz not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_RR", params.sigma_RR_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/pf/sigma_RR not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_PP", params.sigma_PP_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/pf/sigma_pitch not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_YY", params.sigma_YY_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/pf/sigma_YY not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_landmark_matching", params.sigma_landmark_matching_) && !(node_name == "/mapping"
                                                                                                              "_node"))
  {
    ROS_ERROR("%s/pf/sigma_landmark_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_feature_matching", params.sigma_feature_matching_) && !(node_name == "/mapping_"
                                                                                                            "node"))
  {
    ROS_ERROR("%s/pf/sigma_feature_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_corner_matching", params.sigma_corner_matching_) && !(node_name == "/mapping_"
                                                                                                          "node"))
  {
    ROS_ERROR("%s/pf/sigma_corner_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_planar_matching", params.sigma_planar_matching_) && !(node_name == "/mapping_"
                                                                                                          "node"))
  {
    ROS_ERROR("%s/pf/sigma_planar_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_gps", params.sigma_gps_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/pf/sigma_gps not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/k_clusters", params.number_clusters_) && !(node_name == "/mapping_node"))
  {
    ROS_ERROR("%s/pf/k_clusters not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
}

}  // namespace vineslam