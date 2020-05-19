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
  fx         = config["camera_info"]["fx"].as<float>();
  fy         = config["camera_info"]["fy"].as<float>();
  cx         = config["camera_info"]["cx"].as<float>();
  cy         = config["camera_info"]["cy"].as<float>();
  // Load 3D map parameters
  max_range  = config["map_3D"]["max_range"].as<float>();
  max_height = config["map_3D"]["max_height"].as<float>();
}

void Mapper3D::extractFeatures(const cv::Mat& in, std::vector<Feature>& out)
{
  // Array to store the features
  std::vector<cv::KeyPoint> kpts;
  // String to store the type of feature
  std::string type;

  // Perform feature extraction using one of the following
  // feature detectors
#if STAR_ == 1
  type      = "star";
  auto star = cv::xfeatures2d::StarDetector::create(32);
  star->detect(in, kpts);
#elif BRISK_ == 1
  type       = "brisk";
  auto brisk = cv::BRISK::create();
  brisk->detect(in, kpts);
#elif FAST_ == 1
  type      = "fast";
  auto fast = cv::FastFeatureDetector::create();
  fast->detect(in, kpts);
#elif ORB_ == 1
  type     = "orb";
  auto orb = cv::ORB::create(200);
  orb->detect(in, kpts);
#elif KAZE_ == 1
  type      = "kaze";
  auto kaze = cv::KAZE::create();
  kaze->detect(in, kpts);
#elif AKAZE_ == 1
  type       = "akaze";
  auto akaze = cv::AKAZE::create();
  akaze->detect(in, kpts);
#endif

  // Save features in the output array
  for (auto& kpt : kpts) {
    Feature m_ft(kpt.pt.x, kpt.pt.y, type);
    out.push_back(m_ft);
  }
}

void Mapper3D::localMap(const std::vector<Feature>& in_features,
                        std::vector<Feature>&       out_features)
{
}

void Mapper3D::globalMap(const std::vector<Feature>& features,
                         OccupancyMap&               grid_map)
{
}

}; // namespace wildSLAM
