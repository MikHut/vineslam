#include "../include/dense_mapping_node.hpp"

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
  grid_map_ = new OccupancyMap(params_, Pose(0, 0, 0, 0, 0, 0), 1, 1);
  RCLCPP_INFO(this->get_logger(), "Done!");

  // Initialize ICP
  icp_ = new ICP<Planar>();
  icp_->setTolerance(1e-8);
  icp_->setMaxIterations(200);
  icp_->setRejectOutliersFlag(false);

  // Define publishers
  map3D_features_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vineslam/dense_map3D", 10);

  // Call the execution loop
  idx_ = 0;
  loop();
}

void MappingNode::loadParameters(Parameters& params)
{
  std::string prefix = this->get_name();
  std::string param;

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
    std::string val;
    std::vector<float> vals;
    while (std::getline(lstream, val, ','))
    {
      vals.push_back(std::stof(val));
    }

    // Save robot pose
    if (vals.size() != 7)
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(params_.logs_folder_ + "pcd_file_" + std::to_string(idx_) + ".pcd",
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
      points.push_back(f);
    }

    loopOnce(points);
    rclcpp::sleep_for(std::chrono::milliseconds(mil_secs));
    idx_++;
  }
}

void MappingNode::loopOnce(const std::vector<Planar>& points)
{
}

}  // namespace vineslam