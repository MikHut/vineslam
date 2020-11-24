#include "../../include/vineslam/mapping/visual_mapping.hpp"

namespace vineslam
{
VisualMapper::VisualMapper(const Parameters& params)
{
  // Load camera info parameters
  fx_ = params.fx_;
  fy_ = params.fy_;
  cx_ = params.cx_;
  cy_ = params.cy_;
  // Load 3D map parameters
  max_range_ = params.max_range_;
  max_height_ = params.max_height_;
  // Feature detector
  hessian_threshold_ = params.hessian_threshold_;
}

void VisualMapper::registerMaps(const Pose& robot_pose, std::vector<ImageFeature>& img_features, OccupancyMap& grid_map)
{
  // - 3D image feature map estimation
  globalMap(img_features, robot_pose, grid_map);
}

// -------------------------------------------------------------------------------
// ---- 3D image feature map functions
// -------------------------------------------------------------------------------

void VisualMapper::localMap(const cv::Mat& img, const float* depths, std::vector<ImageFeature>& out_features)
{
  // --------- Image feature extraction
  std::vector<ImageFeature> features;
  extractSurfFeatures(img, features);
  // ----------------------------------

  // --------- Build local map of 3D points ----------------------------------------
  for (const auto& feature : features)
  {
    int idx = feature.u_ + img.cols * feature.v_;
    float m_depth = depths[idx];

    // Check validity of depth information
    if (!std::isfinite(depths[idx]))
    {
      continue;
    }

    Point out_pt;
    Point in_pt(static_cast<float>(feature.u_), static_cast<float>(feature.v_), 0.);
    pixel2base(in_pt, m_depth, out_pt);
    // Get the RGB pixel values
    auto* p = img.ptr<cv::Point3_<uchar>>(feature.v_, feature.u_);
    //------------------------------------------------------------------------------
    std::array<uint8_t, 3> c_int = { (*p).z, (*p).y, (*p).x };
    //------------------------------------------------------------------------------
    // Compute feature and insert on grid map
    float dist = std::sqrt((out_pt.x_ * out_pt.x_) + (out_pt.y_ * out_pt.y_) + (out_pt.z_ * out_pt.z_));
    if (out_pt.z_ < max_height_ && dist < max_range_)
    {
      ImageFeature m_feature = feature;
      m_feature.r_ = c_int[0];
      m_feature.g_ = c_int[1];
      m_feature.b_ = c_int[2];
      m_feature.pos_ = out_pt;
      out_features.push_back(m_feature);
    }
  }
  // -------------------------------------------------------------------------------
}

void VisualMapper::globalMap(const std::vector<ImageFeature>& features, const Pose& robot_pose,
                             OccupancyMap& grid_map) const
{
  // ------ Convert robot pose into homogeneous transformation
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // ------ Insert features into the grid map
  for (const auto& image_feature : features)
  {
    // - First convert them to map's referential using the robot pose
    Point m_pt = image_feature.pos_ * tf;

    ImageFeature m_feature = image_feature;
    m_feature.pos_ = m_pt;

    // - Then, look for correspondences in the local map
    ImageFeature correspondence{};
    float best_correspondence = 0.02;
    bool found = false;
    for (const auto& m_image_feature : grid_map(m_pt.x_, m_pt.y_, m_pt.z_).surf_features_)
    {
      float dist_min = m_pt.distance(m_image_feature.pos_);

      if (dist_min < best_correspondence)
      {
        correspondence = m_image_feature;
        best_correspondence = dist_min;
        found = true;
      }
    }

    // Only search in the adjacent cells if we do not find in the source cell
    if (!found)
    {
      std::vector<Cell> adjacents;
      grid_map.getAdjacent(m_pt.x_, m_pt.y_, m_pt.z_, 2, adjacents);
      for (const auto& m_cell : adjacents)
      {
        for (const auto& m_image_feature : m_cell.surf_features_)
        {
          float dist_min = m_pt.distance(m_image_feature.pos_);
          if (dist_min < best_correspondence)
          {
            correspondence = m_image_feature;
            best_correspondence = dist_min;
            found = true;
          }
        }
      }
    }

    // - Then, insert the image feature into the grid map
    if (found)
    {
      Point new_pt = ((correspondence.pos_ * static_cast<float>(correspondence.n_observations_)) + m_pt) /
                     static_cast<float>(correspondence.n_observations_ + 1);
      ImageFeature new_image_feature(image_feature.u_, image_feature.v_, image_feature.r_, image_feature.g_,
                                     image_feature.b_, new_pt);
      new_image_feature.laplacian_ = image_feature.laplacian_;
      new_image_feature.signature_ = image_feature.signature_;
      new_image_feature.n_observations_ = correspondence.n_observations_++;
      grid_map.update(correspondence, new_image_feature);
    }
    else
    {
      ImageFeature new_image_feature(image_feature.u_, image_feature.v_, image_feature.r_, image_feature.g_,
                                     image_feature.b_, m_pt);
      new_image_feature.laplacian_ = image_feature.laplacian_;
      new_image_feature.signature_ = image_feature.signature_;
      grid_map.insert(new_image_feature);
    }
  }
}

void VisualMapper::extractSurfFeatures(const cv::Mat& in, std::vector<ImageFeature>& out) const
{
  using namespace cv::xfeatures2d;

  // Array to store the features
  std::vector<cv::KeyPoint> kpts;
  // String to store the type of feature
  std::string type;
  // Matrix to store the descriptor
  cv::Mat desc;

  // Perform feature extraction
  auto surf = SURF::create(hessian_threshold_);
  surf->detectAndCompute(in, cv::Mat(), kpts, desc);

  // Save features in the output array
  for (auto& kpt : kpts)
  {
    ImageFeature m_ft(kpt.pt.x, kpt.pt.y);
    m_ft.laplacian_ = kpt.class_id;
    out.push_back(m_ft);
  }

  // Save features descriptors
  for (int32_t i = 0; i < desc.rows; i++)
  {
    for (int32_t j = 0; j < desc.cols; j++)
    {
      out[i].signature_.push_back(desc.at<float>(i, j));
    }
  }
}

void VisualMapper::pixel2base(const Point& in_pt, const float& depth, Point& out_pt) const
{
  // Project 2D pixel into a 3D Point using the stereo depth information
  float x_cam = (in_pt.x_ - cx_) * (depth / fx_);
  float y_cam = (in_pt.y_ - cy_) * (depth / fy_);
  float z_cam = depth;
  Point pt_cam(x_cam, y_cam, z_cam);

  // Compute camera-world axis transformation matrix
  Pose cam2world(0., 0., 0, -M_PI / 2., 0., -M_PI / 2.);
  std::array<float, 9> c2w_rot{};
  cam2world.toRotMatrix(c2w_rot);
  Tf cam2world_tf(c2w_rot, std::array<float, 3>{ 0., 0., 0. });

  // Align world and camera axis
  Point wpoint = pt_cam * cam2world_tf;

  // Compute camera-to-base transformation matrix
  Pose cam2base(cam2base_x_, cam2base_y_, cam2base_z_, cam2base_roll_, cam2base_pitch_, cam2base_yaw_);

  std::array<float, 9> c2b_rot{};
  cam2base.toRotMatrix(c2b_rot);
  Tf cam2base_tf(c2b_rot, std::array<float, 3>{ cam2base_x_, cam2base_y_, cam2base_z_ });

  // Transform camera point to base_link
  out_pt = wpoint * cam2base_tf.inverse();
}

}  // namespace vineslam
