#pragma once

#include <ros/ros.h>
#include <params.hpp>

namespace vineslam
{

static void loadParameters(const ros::NodeHandle& nh,
                           const std::string&     prefix,
                           Parameters&            params)
{
  // Load params
  if (!nh.getParam(prefix + "/bagfile_name", params.bagfile_name)) {
    ROS_ERROR("%s/bagfile_name parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/odom_topic", params.odom_topic)) {
    ROS_ERROR("%s/odom_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/rs_odom_topic", params.rs_odom_topic)) {
    ROS_ERROR("%s/rs_odom_topic parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/tf_topic", params.tf_topic)) {
    ROS_ERROR("%s/tf_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/fix_topic", params.fix_topic)) {
    ROS_ERROR("%s/fix_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/depth_img_topic", params.depth_img_topic)) {
    ROS_ERROR("%s/depth_img_topic parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/left_img_topic", params.left_img_topic)) {
    ROS_ERROR("%s/left_img_topic parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/detections_topic", params.detections_topic)) {
    ROS_ERROR("%s/detections_topic parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pcl_topic", params.pcl_topic)) {
    ROS_ERROR("%s/pcl_topic parameter not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/debug", params.debug)) {
    ROS_ERROR("%s/system/debug parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_landmarks", params.use_landmarks)) {
    ROS_ERROR("%s/system/debug parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_vegetation_lines",
                   params.use_vegetation_lines)) {
    ROS_ERROR("%s/system/use_vegetation_lines parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_ground_plane", params.use_ground_plane)) {
    ROS_ERROR("%s/system/use_ground_plane parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_corners", params.use_corners)) {
    ROS_ERROR("%s/system/use_corners parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_icp", params.use_icp)) {
    ROS_ERROR("%s/system/use_icp parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/use_gps", params.use_gps)) {
    ROS_ERROR("%s/system/use_gps parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/gps_datum/lat", params.latitude)) {
    ROS_ERROR("%s/system/gps_datum/lat parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/system/gps_datum/long", params.longitude)) {
    ROS_ERROR("%s/system/gps_datum/long parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/baseline", params.baseline)) {
    ROS_ERROR("%s/camera_info/baseline parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/depth_hfov", params.depth_hfov)) {
    ROS_ERROR("%s/camera_info/depth_hfov parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/depth_vfov", params.depth_vfov)) {
    ROS_ERROR("%s/camera_info/depth_vfov parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/cx", params.cx)) {
    ROS_ERROR("%s/camera_info/cx parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/cy", params.cy)) {
    ROS_ERROR("%s/camera_info/cy parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/fx", params.fx)) {
    ROS_ERROR("%s/camera_info/fx parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/fy", params.fy)) {
    ROS_ERROR("%s/camera_info/fy parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/img_width", params.img_width)) {
    ROS_ERROR("%s/camera_info/img_width parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/camera_info/img_height", params.img_height)) {
    ROS_ERROR("%s/camera_info/img_height parameter not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/ICP/distance_threshold",
                   params.icp_distance_threshold)) {
    ROS_ERROR("%s/multilayer_mapping/ICP/distance_threshold parameter not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/ICP/max_iters",
                   params.icp_max_iters)) {
    ROS_ERROR("%s/multilayer_mapping/ICP/max_iters parameter not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/ICP/reject_outliers",
                   params.icp_reject_outliers)) {
    ROS_ERROR("%s/multilayer_mapping/ICP/max_iters parameter not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/cloud_feature/downsample_factor",
                   params.downsample_factor)) {
    ROS_ERROR("%s/multilayer_mapping/cloud_feature/downsample_factor parameter "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/origin/x",
                   params.gridmap_origin_x)) {
    ROS_ERROR("%s/multilayer_mapping/grid_map/origin/x parameter "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/origin/y",
                   params.gridmap_origin_y)) {
    ROS_ERROR("%s/multilayer_mapping/grid_map/origin/y parameter "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/width",
                   params.gridmap_width)) {
    ROS_ERROR("%s/multilayer_mapping/grid_map/width "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/height",
                   params.gridmap_height)) {
    ROS_ERROR("%s/multilayer_mapping/grid_map/height "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/resolution",
                   params.gridmap_resolution)) {
    ROS_ERROR("%s/multilayer_mapping/grid_map/resolution "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/metric",
                   params.gridmap_metric)) {
    ROS_ERROR("%s/multilayer_mapping/grid_map/metric "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/save_map",
                   params.save_map)) {
    ROS_ERROR("%s/multilayer_mapping/grid_map/save_map "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/output_folder",
                   params.map_output_folder)) {
    ROS_ERROR("%s/multilayer_mapping/grid_map/output_folder "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/grid_map/input_file",
                   params.map_input_file)) {
    ROS_ERROR("%s/multilayer_mapping/grid_map/input_file "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/image_feature/hessian_threshold",
                   params.hessian_threshold)) {
    ROS_ERROR("%s/multilayer_mapping/image_feature/hessian_threshold "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/map_3D/max_range",
                   params.max_range)) {
    ROS_ERROR("%s/multilayer_mapping/map_3D/max_range "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/multilayer_mapping/map_3D/max_height",
                   params.max_height)) {
    ROS_ERROR("%s/multilayer_mapping/map_3D/max_height "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/n_particles", params.number_particles)) {
    ROS_ERROR("%s/pf/n_particles "
              "not found. Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/srr", params.srr)) {
    ROS_ERROR("%s/pf/srr not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/srt", params.srt)) {
    ROS_ERROR("%s/pf/srt not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/str", params.str)) {
    ROS_ERROR("%s/pf/str not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/stt", params.stt)) {
    ROS_ERROR("%s/pf/stt not found. Shutting down...", prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_xy", params.sigma_xy)) {
    ROS_ERROR("%s/pf/sigma_xy not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_z", params.sigma_z)) {
    ROS_ERROR("%s/pf/sigma_z not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_roll", params.sigma_roll)) {
    ROS_ERROR("%s/pf/sigma_roll not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_pitch", params.sigma_pitch)) {
    ROS_ERROR("%s/pf/sigma_pitch not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_yaw", params.sigma_yaw)) {
    ROS_ERROR("%s/pf/sigma_pitch not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_landmark_matching",
                   params.sigma_landmark_matching)) {
    ROS_ERROR("%s/pf/sigma_landmark_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_feature_matching",
                   params.sigma_feature_matching)) {
    ROS_ERROR("%s/pf/sigma_feature_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_corner_matching",
                   params.sigma_corner_matching)) {
    ROS_ERROR("%s/pf/sigma_corner_matching not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_vegetation_lines_yaw",
                   params.sigma_vegetation_lines_yaw)) {
    ROS_ERROR("%s/pf/sigma_vegetation_lines_yaw not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_ground_rp", params.sigma_ground_rp)) {
    ROS_ERROR("%s/pf/sigma_ground_rp not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/sigma_gps", params.sigma_gps)) {
    ROS_ERROR("%s/pf/sigma_gps not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
  if (!nh.getParam(prefix + "/pf/k_clusters", params.number_clusters)) {
    ROS_ERROR("%s/pf/k_clusters not found. "
              "Shutting down...",
              prefix.c_str());
    exit(-1);
  }
}

static void printParameters(const Parameters& params)
{
  std::cout << "-------- VineSLAM parameters log --------" << std::endl;
  std::cout << "-----------------------------------------"
            << std::endl
            << std::endl;

  std::cout << " - ROS topics:" << std::endl;
  std::cout << "   - odom topic: " << params.odom_topic << std::endl;
  std::cout << "   - rsense topic: " << params.rs_odom_topic << std::endl;
  std::cout << "   - fix topic: " << params.fix_topic << std::endl;
  std::cout << "   - tf topic: " << params.tf_topic << std::endl;
  std::cout << "   - depth image topic: " << params.depth_img_topic << std::endl;
  std::cout << "   - left image topic: " << params.left_img_topic << std::endl;
  std::cout << "   - detections topic: " << params.detections_topic << std::endl;
  std::cout << "   - pcl topic: " << params.pcl_topic << std::endl;

  std::cout << " - System parameters:" << std::endl;
  std::cout << "   - debug: " << params.debug << std::endl;
  std::cout << "   - use_vegetation_lines: " << params.use_vegetation_lines << std::endl;
  std::cout << "   - use_ground_plane: " << params.use_ground_plane << std::endl;
  std::cout << "   - use_landmarks: " << params.use_landmarks << std::endl;
  std::cout << "   - use_icp: " << params.use_icp << std::endl;
  std::cout << "   - use_gps: " << params.use_gps << std::endl;

  std::cout << " - GPS datum:" << std::endl;
  std::cout << "   - latitude: " << params.latitude << std::endl;
  std::cout << "   - longitude: " << params.longitude << std::endl;

  std::cout << " - Camera info:" << std::endl;
  std::cout << "   - baseline: " << params.baseline << std::endl;
  std::cout << "   - depth_hfov: " << params.depth_hfov << std::endl;
  std::cout << "   - depth_vfov: " << params.depth_vfov << std::endl;
  std::cout << "   - img_width: " << params.img_width << std::endl;
  std::cout << "   - img_height: " << params.img_height << std::endl;
  std::cout << "   - fx: " << params.fx << std::endl;
  std::cout << "   - fy: " << params.fy << std::endl;
  std::cout << "   - cx: " << params.cx << std::endl;
  std::cout << "   - cy: " << params.cy << std::endl;

  std::cout << " - Multilayer mapping:" << std::endl;
  std::cout << "   - gridmap origin x: " << params.gridmap_origin_x << std::endl;
  std::cout << "   - gridmap origin y: " << params.gridmap_origin_y << std::endl;
  std::cout << "   - gridmap width: " << params.gridmap_width << std::endl;
  std::cout << "   - gridmap height: " << params.gridmap_height << std::endl;
  std::cout << "   - gridmap resolution: " << params.gridmap_resolution << std::endl;
  std::cout << "   - gridmap metric: " << params.gridmap_metric << std::endl;
  std::cout << "   - save map: " << params.save_map << std::endl;
  std::cout << "   - map output folder: " << params.map_output_folder << std::endl;
  std::cout << "   - map input file: " << params.map_input_file << std::endl;
  std::cout << "   - hessian_threshold: " << params.hessian_threshold << std::endl;
  std::cout << "   - downsample_factor: " << params.downsample_factor << std::endl;
  std::cout << "   - max_range: " << params.max_range << std::endl;
  std::cout << "   - max_height: " << params.max_height << std::endl;
  std::cout << "   - icp_max_iters: " << params.icp_max_iters << std::endl;
  std::cout << "   - icp_distance_threshold: " << params.icp_distance_threshold << std::endl;
  std::cout << "   - icp_reject_outliers: " << params.icp_reject_outliers << std::endl;

  std::cout << " - Particle filter:" << std::endl;
  std::cout << "   - n_particles: " << params.number_particles << std::endl;
  std::cout << "   - srr: " << params.srr << std::endl;
  std::cout << "   - srt: " << params.srt << std::endl;
  std::cout << "   - str: " << params.str << std::endl;
  std::cout << "   - stt: " << params.stt << std::endl;
  std::cout << "   - sigma_xy: " << params.sigma_xy << std::endl;
  std::cout << "   - sigma_z: " << params.sigma_z << std::endl;
  std::cout << "   - sigma_roll: " << params.sigma_roll << std::endl;
  std::cout << "   - sigma_pitch: " << params.sigma_pitch << std::endl;
  std::cout << "   - sigma_yaw: " << params.sigma_yaw << std::endl;
  std::cout << "   - sigma_landmark_matching: " << params.sigma_landmark_matching << std::endl;
  std::cout << "   - sigma_corner_matching: " << params.sigma_corner_matching << std::endl;
  std::cout << "   - sigma_feature_matching: " << params.sigma_feature_matching << std::endl;
  std::cout << "   - sigma_vegetation_lines_yaw: " << params.sigma_vegetation_lines_yaw << std::endl;
  std::cout << "   - sigma_ground_rp: " << params.sigma_ground_rp << std::endl;
  std::cout << "   - sigma_gps: " << params.sigma_gps << std::endl;
  std::cout << "   - n_clusters: " << params.number_clusters << std::endl;

  std::cout << "-----------------------------------------" << std::endl;
  std::cout << "-----------------------------------------" << std::endl;
}

} // namespace vineslam