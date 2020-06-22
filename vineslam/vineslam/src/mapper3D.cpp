#include "mapper3D.hpp"

namespace vineslam
{

Mapper3D::Mapper3D(const std::string& config_path)
{
  // Load configuration file
  YAML::Node config = YAML::LoadFile(config_path);

  // Load camera info parameters
  img_width  = config["camera_info"]["img_width"].as<float>();
  img_height = config["camera_info"]["img_height"].as<float>();
  cam_height = config["camera_info"]["cam_height"].as<float>();
  cam_pitch  = config["camera_info"]["cam_pitch"].as<float>() * PI / 180.;
  fx         = config["camera_info"]["fx"].as<float>();
  fy         = config["camera_info"]["fy"].as<float>();
  cx         = config["camera_info"]["cx"].as<float>();
  cy         = config["camera_info"]["cy"].as<float>();
  // Load 3D map parameters
  max_range  = config["map_3D"]["max_range"].as<float>();
  max_height = config["map_3D"]["max_height"].as<float>();
  // Feature detector
  fdetector = config["image_feature"]["type"].as<std::string>();
}

void Mapper3D::localMap(const cv::Mat&        img,
                        float*                depths,
                        std::vector<Feature>& out_features)
{
}

void Mapper3D::globalMap(const std::vector<Feature>& features,
                         const pose&                 robot_pose,
                         OccupancyMap&               grid_map)
{
}

void Mapper3D::featureHandler(const cv::Mat& in, std::vector<Feature>& out)
{
}

void Mapper3D::pixel2world(const point& in_pt,
                           const float& depth,
                           point&       out_pt) const
{
  // Project 2D pixel into a 3D Point using the stereo depth information
  float x_cam = static_cast<float>((in_pt.x - cx) * (depth / fx));
  float y_cam = static_cast<float>((in_pt.y - cy) * (depth / fy));
  float z_cam = depth;

  // Compute camera-world axis transformation matrix
  // - NOTE: We compensate here the camera height and pitch (!)
  pose transform(0., 0., cam_height, -PI / 2. - cam_pitch, 0., -PI / 2.);
  std::array<float, 9> c2w_rot = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
  transform.toRotMatrix(c2w_rot);

  // Align world and camera axis
  out_pt.x =
      c2w_rot[0] * x_cam + c2w_rot[1] * y_cam + c2w_rot[2] * z_cam + transform.x;
  out_pt.y =
      c2w_rot[3] * x_cam + c2w_rot[4] * y_cam + c2w_rot[5] * z_cam + transform.y;
  out_pt.z =
      c2w_rot[6] * x_cam + c2w_rot[7] * y_cam + c2w_rot[8] * z_cam + transform.z;
}

}; // namespace vineslam
