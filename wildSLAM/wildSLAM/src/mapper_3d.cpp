#include "mapper_3d.hpp"

Mapper3D::Mapper3D(const std::string& config_path)
{
  // Load configuration file
  YAML::Node config = YAML::LoadFile(config_path);

  // Load camera info parameters
  img_width  = config["camera_info"]["img_width"].as<float>();
  img_height = config["camera_info"]["img_height"].as<float>();
  cam_height = config["camera_info"]["cam_height"].as<float>();
  fx         = config["camera_info"]["fx"].as<float>();
  fy         = config["camera_info"]["fy"].as<float>();
  cx         = config["camera_info"]["cx"].as<float>();
  cy         = config["camera_info"]["cy"].as<float>();
}
