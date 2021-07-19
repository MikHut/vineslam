#include "../../include/offline/dense_mapping_node.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vineslam::MappingNode>());
  rclcpp::shutdown();

  return 0;
}

namespace vineslam
{
MappingNode::MappingNode() : VineSLAM_ros("MappingNode")
{
  // Load parameters
  loadParameters(params_);

  // Allocate map memory
  RCLCPP_INFO(this->get_logger(), "Allocating map memory!");
  grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0), 5, 1);
  elevation_map_ = new ElevationMap(params_, Pose(0, 0, 0, 0, 0, 0));
  RCLCPP_INFO(this->get_logger(), "Done!");

  // Extract SLAM map info
  RCLCPP_INFO(this->get_logger(), "Extracting SLAM map information.");
  OccupancyMap* l_grid_map;
  Parameters l_params;
  l_params.map_input_file_ = params_.map_input_file_;
  MapParser map_parser(l_params);

  if (!map_parser.parseHeader(&l_params))
  {
    RCLCPP_ERROR(this->get_logger(), "Map input file not found.");
    return;
  }
  else
  {
    l_grid_map = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 1, 1);
  }

  if (!map_parser.parseFile(&(*l_grid_map)))
  {
    RCLCPP_ERROR(this->get_logger(), "Map input file not found.");
    return;
  }

  // Parse the necessary parameters from the loaded grid map
  params_.map_datum_lat_ = l_params.map_datum_lat_;
  params_.map_datum_long_ = l_params.map_datum_long_;
  params_.map_datum_alt_ = l_params.map_datum_alt_;
  params_.map_datum_head_ = l_params.map_datum_head_;

  semantic_features_ = l_grid_map->getLandmarks();
  free(l_grid_map);
  RCLCPP_INFO(this->get_logger(), "Done!");

  // Define publishers
  poses_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/poses", 10);
  map3D_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/dense_map3D", 10);
  mesh_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/vineslam/mesh", 10);
  semantic_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vineslam/semantic_map", 10);

  // Define services
  save_map_srv_ = this->create_service<vineslam_ros::srv::SaveMap>(
      "/vineslam/save_map", std::bind(&VineSLAM_ros::saveMap, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1,
                                      std::placeholders::_2));

  // Static transforms
  RCLCPP_INFO(this->get_logger(), "Waiting for static transforms...");
  tf2_ros::Buffer tf_buffer(this->get_clock());
  tf2_ros::TransformListener tfListener(tf_buffer);
  geometry_msgs::msg::TransformStamped laser2base_msg;
  bool got_laser2base = false;
  while (!got_laser2base && rclcpp::ok())
  {
    try
    {
      laser2base_msg = tf_buffer.lookupTransform(params_.lidar_sensor_frame_, "base_link", rclcpp::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
      continue;
    }
    got_laser2base = true;
  }
  tf2::Stamped<tf2::Transform> laser2base_stamped;
  tf2::fromMsg(laser2base_msg, laser2base_stamped);

  tf2::Transform laser2base = laser2base_stamped;  //.inverse();
  tf2Scalar roll, pitch, yaw;
  tf2::Vector3 t = laser2base.getOrigin();
  laser2base.getBasis().getRPY(roll, pitch, yaw);

  laser_to_base_tf_ = Pose(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw).toTf();
  RCLCPP_INFO(this->get_logger(), "Received!");

  // Call the execution loop
  idx_ = 0;
  std::thread th(&MappingNode::loop, this);
  th.detach();
}

void MappingNode::loadParameters(Parameters& params)
{
  std::string prefix = this->get_name();
  std::string param;

  param = prefix + ".world_frame_id";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.world_frame_id_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".logs_folder";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.logs_folder_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".lidar_sensor_frame";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.lidar_sensor_frame_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.datum.latitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_lat_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.datum.longitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_long_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.datum.altitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_alt_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.map_file_path";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_input_file_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.origin.x";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_origin_x_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.origin.y";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_origin_y_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.origin.z";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_origin_z_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.width";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_width_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.lenght";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_lenght_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.height";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_height_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.resolution";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_resolution_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = prefix + ".multilayer_mapping.grid_map.output_folder";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_output_folder_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
}

void MappingNode::loop()
{
  // Define loop rate
  uint32_t rate = 3;
  uint32_t mil_secs = static_cast<uint32_t>((1 / rate) * 1e3);

  // Open input file and go through it
  std::ifstream file(params_.logs_folder_ + "vineslam_logs.txt");
  std::string line;

  // Create markers to save poses
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.ns = "/poses";
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.4;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.header.frame_id = params_.world_frame_id_;

  while (rclcpp::ok() && std::getline(file, line))
  {
    RCLCPP_INFO(this->get_logger(), "Processing point cloud number %d.", idx_);

    std::stringstream lstream(line);

    float val;
    std::vector<float> vals;
    while (lstream >> val)
    {
      vals.push_back(val);
    }

    // Save robot pose
    if (vals.size() != 6)
    {
      RCLCPP_ERROR(this->get_logger(), "Problem reading input file, wrong number of inputs per line.");
      break;
    }
    else
    {
      robot_pose_.x_ = vals[0];
      robot_pose_.y_ = vals[1];
      robot_pose_.z_ = vals[2];
      robot_pose_.R_ = vals[3];
      robot_pose_.P_ = vals[4];
      robot_pose_.Y_ = vals[5];
    }

    // Publish pose
    tf2::Quaternion q;
    q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);
    q.normalize();
    marker.id = idx_;
    marker.pose.position.x = robot_pose_.x_;
    marker.pose.position.y = robot_pose_.y_;
    marker.pose.position.z = robot_pose_.z_;
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();
    marker_array.markers.push_back(marker);
    poses_publisher_->publish(marker_array);

    // Publish landmarks
    publishSemanticMapFromArray(semantic_features_);

    // Read pcd file
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(params_.logs_folder_ + "pcl_file_" + std::to_string(idx_) + ".pcd",
                                             *cloud) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not read pcd file.");
      break;
    }

    // Convert from pcl to vineslam
    std::vector<Planar> points;
    for (const auto& pt : *cloud)
    {
      Planar f;
      f.pos_.x_ = pt.x;
      f.pos_.y_ = pt.y;
      f.pos_.z_ = pt.z;
      f.pos_.intensity_ = pt.intensity;

      // We don't want to map point close to the robot
      // this deletes noise (points that belong to the robot structure, and pedestrians near the robot)
      if (f.pos_.norm3D() < 2.0)
      {
        continue;
      }

      f.pos_ = f.pos_ * laser_to_base_tf_.inverse();
      points.push_back(f);
    }

    loopOnce(points);
    rclcpp::sleep_for(std::chrono::milliseconds(mil_secs));
    idx_++;
  }

  // Convert the map to a 3D mesh
  std::vector<Planar> points = grid_map_->getPlanars();
  pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
  for (const auto& pt : points)
  {
    pcl::PointXYZI pcl_pt;
    pcl_pt.x = pt.pos_.x_;
    pcl_pt.y = pt.pos_.y_;
    pcl_pt.z = pt.pos_.z_;
    pcl_pt.intensity = pt.pos_.intensity_;
    map->push_back(pcl_pt);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*map, *map_xyz);
  pcl::PolygonMesh mesh;
  cloudToMesh(map_xyz, mesh);

  // Save map and mesh to files
  pcl::io::savePLYFile(params_.map_output_folder_ + "map.ply", *map);
  pcl::io::savePLYFile(params_.map_output_folder_ + "mesh.ply", mesh);

  // Publish the poses, map and mesh continuously
  visualization_msgs::msg::Marker ros_mesh;
  meshToMarkerMsg(mesh, ros_mesh);

  map->header.frame_id = params_.world_frame_id_;
  sensor_msgs::msg::PointCloud2 map2;
  pcl::toROSMsg(*map, map2);
  map3D_publisher_->publish(map2);

  while (rclcpp::ok())
  {
    poses_publisher_->publish(marker_array);
    mesh_publisher_->publish(ros_mesh);
    map3D_publisher_->publish(map2);
    publishSemanticMapFromArray(semantic_features_);
    rclcpp::sleep_for(std::chrono::milliseconds(mil_secs));
  }
}

void MappingNode::loopOnce(const std::vector<Planar>& points)
{
  // Insert points into the map
  registerPoints(robot_pose_, points, *grid_map_);

  // // Push back points
  // pcl::PointCloud<pcl::PointXYZI>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // std::vector<Planar> all_points = grid_map_->getPlanars();
  // for (auto point : all_points)
  // {
  //   pcl::PointXYZI l_pt;
  //   l_pt.x = point.pos_.x_;
  //   l_pt.y = point.pos_.y_;
  //   l_pt.z = point.pos_.z_;
  //   l_pt.intensity = point.pos_.intensity_;

  //   planar_cloud->points.push_back(l_pt);
  // }

  // // Publish cloud
  // planar_cloud->header.frame_id = params_.world_frame_id_;
  // sensor_msgs::msg::PointCloud2 planar_cloud2;
  // pcl::toROSMsg(*planar_cloud, planar_cloud2);
  // map3D_publisher_->publish(planar_cloud2);
}

void MappingNode::registerPoints(Pose robot_pose, const std::vector<Planar>& points, OccupancyMap& grid_map)
{
  // ----------------------------------------------------------------------------
  // ------ Convert robot pose into homogeneous transformation
  // ----------------------------------------------------------------------------
  Tf tf = robot_pose.toTf();

  // Local array to store the new planar features
  std::vector<Planar> new_points;

  // ----------------------------------------------------------------------------
  // ------ Insert planar into the grid map
  // ----------------------------------------------------------------------------
  for (auto& point : points)
  {
    // - First convert them to map's referential using the robot pose
    Point l_pt = point.pos_ * tf;
    l_pt.intensity_ = point.pos_.intensity_;

    // - Then, look for correspondences in the local map
    Planar correspondence{};
    float best_correspondence = 0.05;
    bool found = false;
    std::vector<Planar>* l_points = { nullptr };

    if (!grid_map.isInside(l_pt.x_, l_pt.y_, l_pt.z_))
    {
      continue;
    }

    if (grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data != nullptr)
    {
      l_points = grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_;
    }

    if (l_points != nullptr)
    {
      for (const auto& l_point : *l_points)
      {
        float dist_min = l_pt.distance(l_point.pos_);

        if (dist_min < best_correspondence)
        {
          correspondence = l_point;
          best_correspondence = dist_min;
          found = true;
        }
      }

      found &= (best_correspondence < 0.05);
    }

    // - Then, insert the planar into the grid map
    if (found)
    {
      Point new_pt = ((correspondence.pos_ * static_cast<float>(correspondence.n_observations_)) + l_pt) /
                     static_cast<float>(correspondence.n_observations_ + 1);
      new_pt.intensity_ = l_pt.intensity_;
      Planar new_point(new_pt, point.pos_.intensity_);
      new_point.n_observations_ = correspondence.n_observations_ + 1;
      grid_map.update(correspondence, new_point);
    }
    else
    {
      Planar new_point(l_pt, point.pos_.intensity_);
      new_points.push_back(new_point);
    }
  }

  // Insert the new observations found
  for (const auto& point : new_points)
    grid_map.insert(point);
}

void MappingNode::cloudToMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh& mesh)
{
  double voxel_size = 0.10;

  boost::shared_ptr<pcl::PointCloud<pcl::PointNormal>> point_cloud_normal_voxelized(
      new pcl::PointCloud<pcl::PointNormal>());

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_voxelized(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> pre_filter;
  pre_filter.setInputCloud(cloud);
  pre_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  pre_filter.filter(*pc_voxelized);

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setSearchRadius(0.25);
  mls.setPolynomialOrder(1);
  mls.setComputeNormals(true);
  mls.setInputCloud(pc_voxelized);

  boost::shared_ptr<pcl::PointCloud<pcl::PointNormal>> point_cloud_mls_normal;
  point_cloud_mls_normal.reset(new pcl::PointCloud<pcl::PointNormal>);
  mls.process(*point_cloud_mls_normal);

  pcl::VoxelGrid<pcl::PointNormal> filter;
  filter.setInputCloud(point_cloud_mls_normal);
  filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  filter.filter(*point_cloud_normal_voxelized);

  boost::shared_ptr<pcl::search::KdTree<pcl::PointNormal>> tree2(new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(point_cloud_normal_voxelized);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> greedy;
  greedy.setSearchRadius(0.25);
  greedy.setMu(2.5);
  greedy.setMaximumNearestNeighbors(50);
  greedy.setMinimumAngle(M_PI / 18);     // 10 degrees
  greedy.setMaximumAngle(2 * M_PI / 3);  // 120 degrees
  greedy.setNormalConsistency(true);
  greedy.setConsistentVertexOrdering(true);

  greedy.setSearchMethod(tree2);
  greedy.setInputCloud(point_cloud_normal_voxelized);

  greedy.reconstruct(mesh);
}

void MappingNode::meshToShapeMsg(const pcl::PolygonMesh& in, shape_msgs::msg::Mesh& mesh)
{
  pcl_msgs::msg::PolygonMesh pcl_msg_mesh;
  pcl_conversions::fromPCL(in, pcl_msg_mesh);

  sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

  size_t size = pcd_modifier.size();
  mesh.vertices.resize(size);
  sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");
  for (size_t i = 0; i < size; i++, ++pt_iter)
  {
    mesh.vertices[i].x = pt_iter[0];
    mesh.vertices[i].y = pt_iter[1];
    mesh.vertices[i].z = pt_iter[2];
  }

  mesh.triangles.resize(in.polygons.size());
  for (size_t i = 0; i < in.polygons.size(); ++i)
  {
    if (in.polygons[i].vertices.size() < 3)
    {
      RCLCPP_INFO(this->get_logger(), "Not enough points in polygon. Ignoring it.");
      continue;
    }

    for (int j = 0; j < 3; ++j)
      mesh.triangles[i].vertex_indices[j] = in.polygons[i].vertices[j];
  }
}

void MappingNode::meshToMarkerMsg(const pcl::PolygonMesh& in, visualization_msgs::msg::Marker& marker)
{
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.header.frame_id = params_.world_frame_id_;
  marker.header.stamp = rclcpp::Time();
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.id = 1;
  marker.action = visualization_msgs::msg::Marker::ADD;

  shape_msgs::msg::Mesh shape_msg_mesh;
  meshToShapeMsg(in, shape_msg_mesh);

  size_t size_triangles = shape_msg_mesh.triangles.size();
  marker.points.resize(size_triangles * 3);
  size_t i = 0;
  for (size_t tri_index = 0; tri_index < size_triangles; ++tri_index)
  {
    marker.points[i] = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[0]];
    marker.points[i + 1] = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[1]];
    marker.points[i + 2] = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[2]];
    i = i + 3;
  }
}

}  // namespace vineslam