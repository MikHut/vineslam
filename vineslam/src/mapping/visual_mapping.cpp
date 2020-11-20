#include "../../include/vineslam/mapping/visual_mapping.hpp"

namespace vineslam
{

VisualMapper::VisualMapper(const Parameters& params)
{
  // Load camera info parameters
  img_width  = params.img_width;
  img_height = params.img_height;
  fx         = params.fx;
  fy         = params.fy;
  cx         = params.cx;
  cy         = params.cy;
  depth_hfov = params.depth_hfov;
  depth_vfov = params.depth_vfov;
  // Load 3D map parameters
  max_range  = params.max_range;
  max_height = params.max_height;
  // Feature detector
  hessian_threshold = params.hessian_threshold;
}

void VisualMapper::registerMaps(const pose&                robot_pose,
                                std::vector<ImageFeature>& img_features,
                                OccupancyMap&              grid_map)
{
  // - 3D image feature map estimation
  globalMap(img_features, robot_pose, grid_map);
}

// -------------------------------------------------------------------------------
// ---- 3D image feature map functions
// -------------------------------------------------------------------------------

void VisualMapper::localMap(const cv::Mat&             img,
                            const float*               depths,
                            std::vector<ImageFeature>& out_features)
{
  // --------- Image feature extraction
  std::vector<ImageFeature> features;
  extractSurfFeatures(img, features);
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
    pixel2base(in_pt, m_depth, out_pt);
    // Get the RGB pixel values
    auto* p = img.ptr<cv::Point3_<uchar>>(feature.v, feature.u);
    //------------------------------------------------------------------------------
    std::array<uint8_t, 3> c_int = {(*p).z, (*p).y, (*p).x};
    //------------------------------------------------------------------------------
    // Compute feature and insert on grid map
    float dist = std::sqrt((out_pt.x * out_pt.x) + (out_pt.y * out_pt.y) +
                           (out_pt.z * out_pt.z));
    if (out_pt.z < max_height && dist < max_range) {
      ImageFeature m_feature = feature;
      m_feature.r            = c_int[0];
      m_feature.g            = c_int[1];
      m_feature.b            = c_int[2];
      m_feature.pos          = out_pt;
      out_features.push_back(m_feature);
    }
  }
  // -------------------------------------------------------------------------------
}

void VisualMapper::globalMap(const std::vector<ImageFeature>& features,
                             const pose&                      robot_pose,
                             OccupancyMap&                    grid_map) const
{
  // ------ Convert robot pose into homogeneous transformation
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = {robot_pose.x, robot_pose.y, robot_pose.z};
  TF                   tf(Rot, trans);

  // ------ Insert features into the grid map
  for (const auto& image_feature : features) {
    // - First convert them to map's referential using the robot pose
    point m_pt = image_feature.pos * tf;

    ImageFeature m_feature = image_feature;
    m_feature.pos          = m_pt;

    // - Then, look for correspondences in the local map
    ImageFeature correspondence{};
    float        best_correspondence = 0.02;
    bool         found               = false;
    for (const auto& m_image_feature :
         grid_map(m_pt.x, m_pt.y, m_pt.z).surf_features) {
      float dist_min = m_pt.distance(m_image_feature.pos);

      if (dist_min < best_correspondence) {
        correspondence      = m_image_feature;
        best_correspondence = dist_min;
        found               = true;
      }
    }

    // Only search in the adjacent cells if we do not find in the source cell
    if (!found) {
      std::vector<Cell> adjacents;
      grid_map.getAdjacent(m_pt.x, m_pt.y, m_pt.z, 2, adjacents);
      for (const auto& m_cell : adjacents) {
        for (const auto& m_image_feature : m_cell.surf_features) {
          float dist_min = m_pt.distance(m_image_feature.pos);
          if (dist_min < best_correspondence) {
            correspondence      = m_image_feature;
            best_correspondence = dist_min;
            found               = true;
          }
        }
      }
    }

    // - Then, insert the image feature into the grid map
    if (found) {
      point new_pt =
          ((correspondence.pos * static_cast<float>(correspondence.n_observations)) +
           m_pt) /
          static_cast<float>(correspondence.n_observations + 1);
      ImageFeature new_image_feature(image_feature.u,
                                     image_feature.v,
                                     image_feature.r,
                                     image_feature.g,
                                     image_feature.b,
                                     new_pt);
      new_image_feature.laplacian      = image_feature.laplacian;
      new_image_feature.signature      = image_feature.signature;
      new_image_feature.n_observations = correspondence.n_observations++;
      grid_map.update(correspondence, new_image_feature);
    } else {
      ImageFeature new_image_feature(image_feature.u,
                                     image_feature.v,
                                     image_feature.r,
                                     image_feature.g,
                                     image_feature.b,
                                     m_pt);
      new_image_feature.laplacian = image_feature.laplacian;
      new_image_feature.signature = image_feature.signature;
      grid_map.insert(new_image_feature);
    }
  }
}

void VisualMapper::extractSurfFeatures(const cv::Mat&             in,
                                       std::vector<ImageFeature>& out) const
{
  using namespace cv::xfeatures2d;

  // Array to store the features
  std::vector<cv::KeyPoint> kpts;
  // String to store the type of feature
  std::string type;
  // Matrix to store the descriptor
  cv::Mat desc;

  // Perform feature extraction
  auto surf = SURF::create(hessian_threshold);
  surf->detectAndCompute(in, cv::Mat(), kpts, desc);

  // Save features in the output array
  for (auto& kpt : kpts) {
    ImageFeature m_ft(kpt.pt.x, kpt.pt.y);
    m_ft.laplacian = kpt.class_id;
    out.push_back(m_ft);
  }

  // Save features descriptors
  for (int32_t i = 0; i < desc.rows; i++) {
    for (int32_t j = 0; j < desc.cols; j++) {
      out[i].signature.push_back(desc.at<float>(i, j));
    }
  }
}

void VisualMapper::pixel2base(const point& in_pt,
                              const float& depth,
                              point&       out_pt) const
{
  // Project 2D pixel into a 3D Point using the stereo depth information
  float x_cam = (in_pt.x - cx) * (depth / fx);
  float y_cam = (in_pt.y - cy) * (depth / fy);
  float z_cam = depth;
  point pt_cam(x_cam, y_cam, z_cam);

  // Compute camera-world axis transformation matrix
  pose                 cam2world(0., 0., 0, -M_PI / 2., 0., -M_PI / 2.);
  std::array<float, 9> c2w_rot{};
  cam2world.toRotMatrix(c2w_rot);
  TF cam2world_tf(c2w_rot, std::array<float, 3>{0., 0., 0.});

  // Align world and camera axis
  point wpoint = pt_cam * cam2world_tf;

  // Compute camera-to-base transformation matrix
  pose cam2base(cam2base_x,
                cam2base_y,
                cam2base_z,
                cam2base_roll,
                cam2base_pitch,
                cam2base_yaw);

  std::array<float, 9> c2b_rot{};
  cam2base.toRotMatrix(c2b_rot);
  TF cam2base_tf(c2b_rot, std::array<float, 3>{cam2base_x, cam2base_y, cam2base_z});

  // Transform camera point to base_link
  out_pt = wpoint * cam2base_tf.inverse();
}

} // namespace vineslam
