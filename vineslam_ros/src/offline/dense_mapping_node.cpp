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
  grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0), 20, 1);
  RCLCPP_INFO(this->get_logger(), "Done!");

  // Initialize ICP
  icp_ = new ICP<Planar>();
  icp_->setTolerance(1e-4);
  icp_->setMaxIterations(200);
  icp_->setRejectOutliersFlag(false);

  // Define publishers
  map3D_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/dense_map3D", 10);

  // Call the execution loop
  idx_ = 0;
  loop();
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
  while (rclcpp::ok() && std::getline(file, line))
  {
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
      points.push_back(f);
    }

    loopOnce(points);
    rclcpp::sleep_for(std::chrono::milliseconds(mil_secs));
    idx_++;
  }
}

void MappingNode::loopOnce(const std::vector<Planar>& points)
{
  // Insert points into the map
  registerPoints(robot_pose_, points, *grid_map_);

  // Push back points
  pcl::PointCloud<pcl::PointXYZI>::Ptr planar_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<Planar> all_points = grid_map_->getPlanars();
  for (auto point : all_points)
  {
    pcl::PointXYZI l_pt;
    l_pt.x = point.pos_.x_;
    l_pt.y = point.pos_.y_;
    l_pt.z = point.pos_.z_;
    l_pt.intensity = point.pos_.intensity_;

    planar_cloud->points.push_back(l_pt);
  }

  // Publish cloud
  planar_cloud->header.frame_id = params_.world_frame_id_;
  sensor_msgs::msg::PointCloud2 planar_cloud2;
  pcl::toROSMsg(*planar_cloud, planar_cloud2);
  map3D_publisher_->publish(planar_cloud2);
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

}  // namespace vineslam