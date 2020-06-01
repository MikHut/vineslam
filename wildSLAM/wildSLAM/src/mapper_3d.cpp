#include "mapper_3d.hpp"

namespace wildSLAM
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
  // --------- Image feature extraction
  std::vector<Feature> features;
  featureHandler(img, features);
  // ----------------------------------

  // --------- Build local map of 3D points ----------------------------------------
  for (const auto& feature : features) {
    int   idx     = feature.u + img.cols * feature.v;
    float m_depth = depths[idx];

    // Check validity of depth information
    if (!std::isfinite(depths[idx])) {
      continue;
    }

    point out_pt;
    point in_pt(static_cast<float>(feature.u), static_cast<float>(feature.v), 0.);
    pixel2world(in_pt, m_depth, out_pt);
    // Get the RGB pixel values
    auto* p = img.ptr<cv::Point3_<uchar>>(feature.v, feature.u);
    //------------------------------------------------------------------------------
    std::array<uint8_t, 3> c_int = {(*p).z, (*p).y, (*p).x};
    //------------------------------------------------------------------------------
    // Compute feature and insert on grid map
    float dist =
        sqrt((out_pt.x * out_pt.x) + (out_pt.y * out_pt.y) + (out_pt.z * out_pt.z));
    if (out_pt.z < max_height && dist < max_range) {
      Feature m_feature = feature;
      m_feature.r       = c_int[0];
      m_feature.g       = c_int[1];
      m_feature.b       = c_int[2];
      m_feature.pos     = out_pt;
      out_features.push_back(m_feature);
    }
  }
  // -------------------------------------------------------------------------------
}

void Mapper3D::globalMap(const std::vector<Feature>& features,
                         const pose&                 robot_pose,
                         OccupancyMap&               grid_map)
{
  // ------ Convert robot pose into homogeneous transformation
  std::array<float, 9> Rot = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = {robot_pose.x, robot_pose.y, robot_pose.z};

  // ------ Insert features into the grid map
  for (const auto& feature : features) {
    // - First convert them to map's referential using the robot pose
    Feature m_feature = feature;

    point m_pt;
    m_pt.x = feature.pos.x * Rot[0] + feature.pos.y * Rot[1] +
             feature.pos.z * Rot[2] + trans[0];
    m_pt.y = feature.pos.x * Rot[3] + feature.pos.y * Rot[4] +
             feature.pos.z * Rot[5] + trans[1];
    m_pt.z = feature.pos.x * Rot[6] + feature.pos.y * Rot[7] +
             feature.pos.z * Rot[8] + trans[2];

    m_feature.pos = m_pt;

    // - Then, insert the feature into the grid map
    grid_map.insert(m_feature, m_feature.pos.x, m_feature.pos.y);
  }
}

void Mapper3D::featureHandler(const cv::Mat& in, std::vector<Feature>& out)
{
  using namespace cv::xfeatures2d;

  // Array to store the features
  std::vector<cv::KeyPoint> kpts;
  // String to store the type of feature
  std::string type;
  // Matrix to store the descriptor
  cv::Mat desc;

  // Perform feature extraction
  if (fdetector == "SIFT") {
    type      = "SIFT";
    auto sift = SIFT::create(1000);
    sift->detectAndCompute(in, cv::Mat(), kpts, desc);
  } else {
    type      = "SURF";
    auto surf = SURF::create(1800);
    surf->detectAndCompute(in, cv::Mat(), kpts, desc);
  }

  // Save features in the output array
  for (auto& kpt : kpts) {
    Feature m_ft(kpt.pt.x, kpt.pt.y, type);
    m_ft.laplacian = kpt.class_id;
    out.push_back(m_ft);
  }

  // Save features descriptors
  for (int32_t i = 0; i < desc.rows; i++) {
    for (int32_t j = 0; j < desc.cols; j++) {
      out[i].desc.push_back(desc.at<float>(i, j));
    }
  }
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

}; // namespace wildSLAM
