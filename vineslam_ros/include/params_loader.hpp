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
  if (!nh.getParam(prefix + "/tf_topic", params.tf_topic_))
  {
    ROS_ERROR("%s/tf_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/fix_topic", params.fix_topic_))
  {
    ROS_ERROR("%s/fix_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/depth_img_topic", params.depth_img_topic_))
  {
    ROS_ERROR("%s/depth_img_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/left_img_topic", params.left_img_topic_))
  {
    ROS_ERROR("%s/left_img_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/detections_topic", params.detections_topic_))
  {
    ROS_ERROR("%s/detections_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pcl_topic", params.pcl_topic_))
  {
    ROS_ERROR("%s/pcl_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_landmarks", params.use_landmarks_))
  {
    ROS_ERROR("%s/system/debug parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_corners", params.use_corners_))
  {
    ROS_ERROR("%s/system/use_corners parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_planars", params.use_planars_))
  {
    ROS_ERROR("%s/system/use_planars parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_icp", params.use_icp_))
  {
    ROS_ERROR("%s/system/use_icp parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_gps", params.use_gps_))
  {
    ROS_ERROR("%s/system/use_gps parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/gps_datum/lat", params.latitude_))
  {
    ROS_ERROR("%s/system/gps_datum/lat parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/gps_datum/long", params.longitude_))
  {
    ROS_ERROR("%s/system/gps_datum/long parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/cam2base", params.cam2base_))
  {
    ROS_ERROR("%s/system/cam2base parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/vel2base", params.vel2base_))
  {
    ROS_ERROR("%s/system/vel2base parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/baseline", params.baseline_))
  {
    ROS_ERROR("%s/camera_info/baseline parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/depth_hfov", params.depth_hfov_))
  {
    ROS_ERROR("%s/camera_info/depth_hfov parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/depth_vfov", params.depth_vfov_))
  {
    ROS_ERROR("%s/camera_info/depth_vfov parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/cx", params.cx_))
  {
    ROS_ERROR("%s/camera_info/cx parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/cy", params.cy_))
  {
    ROS_ERROR("%s/camera_info/cy parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/fx", params.fx_))
  {
    ROS_ERROR("%s/camera_info/fx parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/fy", params.fy_))
  {
    ROS_ERROR("%s/camera_info/fy parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/img_width", params.img_width_))
  {
    ROS_ERROR("%s/camera_info/img_width parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/img_height", params.img_height_))
  {
    ROS_ERROR("%s/camera_info/img_height parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/ICP/distance_threshold", params.icp_distance_threshold_))
  {
    ROS_ERROR("%s/multilayer_mapping/ICP/distance_threshold parameter not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/ICP/max_iters", params.icp_max_iters_))
  {
    ROS_ERROR("%s/multilayer_mapping/ICP/max_iters parameter not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/ICP/reject_outliers", params.icp_reject_outliers_))
  {
    ROS_ERROR("%s/multilayer_mapping/ICP/max_iters parameter not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/origin/x", params.gridmap_origin_x_))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/origin/x parameter "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/origin/y", params.gridmap_origin_y_))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/origin/y parameter "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/origin/z", params.gridmap_origin_z_))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/origin/z parameter "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/width", params.gridmap_width_))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/width "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/lenght", params.gridmap_lenght_))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/lenght "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/height", params.gridmap_height_))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/height "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/resolution", params.gridmap_resolution_))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/resolution "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/save_map", params.save_map_))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/save_map "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/output_folder", params.map_output_folder_))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/output_folder "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/input_file", params.map_input_file_))
  {
    ROS_ERROR("%s/multilayer_mapping/grid_map/input_file "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/image_feature/hessian_threshold", params.hessian_threshold_))
  {
    ROS_ERROR("%s/multilayer_mapping/image_feature/hessian_threshold "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/map_3D/max_range", params.max_range_))
  {
    ROS_ERROR("%s/multilayer_mapping/map_3D/max_range "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/map_3D/max_height", params.max_height_))
  {
    ROS_ERROR("%s/multilayer_mapping/map_3D/max_height "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/n_particles", params.number_particles_))
  {
    ROS_ERROR("%s/pf/n_particles "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_xx", params.sigma_xx_))
  {
    ROS_ERROR("%s/pf/sigma_xx not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_yy", params.sigma_yy_))
  {
    ROS_ERROR("%s/pf/sigma_yy not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_zz", params.sigma_zz_))
  {
    ROS_ERROR("%s/pf/sigma_zz not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_RR", params.sigma_RR_))
  {
    ROS_ERROR("%s/pf/sigma_RR not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_PP", params.sigma_PP_))
  {
    ROS_ERROR("%s/pf/sigma_pitch not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_YY", params.sigma_YY_))
  {
    ROS_ERROR("%s/pf/sigma_YY not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_landmark_matching", params.sigma_landmark_matching_))
  {
    ROS_ERROR("%s/pf/sigma_landmark_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_feature_matching", params.sigma_feature_matching_))
  {
    ROS_ERROR("%s/pf/sigma_feature_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_corner_matching", params.sigma_corner_matching_))
  {
    ROS_ERROR("%s/pf/sigma_corner_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_planar_matching", params.sigma_planar_matching_))
  {
    ROS_ERROR("%s/pf/sigma_planar_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_gps", params.sigma_gps_))
  {
    ROS_ERROR("%s/pf/sigma_gps not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/k_clusters", params.number_clusters_))
  {
    ROS_ERROR("%s/pf/k_clusters not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
}

}  // namespace vineslam