#include "../../include/vineslam/mapping/visual_mapping.hpp"

namespace vineslam
{
VisualMapper::VisualMapper(const Parameters& params)
{
}

void VisualMapper::registerMaps(const Pose& robot_pose, std::vector<ImageFeature>& img_features, OccupancyMap& grid_map)
{
  // - 3D image feature map estimation
  globalMap(img_features, robot_pose, grid_map);
}

// -------------------------------------------------------------------------------
// ---- 3D image feature map functions
// -------------------------------------------------------------------------------

void VisualMapper::localMap(const std::vector<ImageFeature>& in_features, std::vector<ImageFeature>& out_features)
{
  for (const auto& in_feature : in_features)
  {
    ImageFeature l_feature = in_feature;
    pixel2base(l_feature.pos_, l_feature.pos_);
    out_features.push_back(l_feature);
  }
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
    //    if (found)
    //    {
    //      Point new_pt = ((correspondence.pos_ * static_cast<float>(correspondence.n_observations_)) + m_pt) /
    //                     static_cast<float>(correspondence.n_observations_ + 1);
    //      ImageFeature new_image_feature(image_feature.u_, image_feature.v_, image_feature.r_, image_feature.g_,
    //                                     image_feature.b_, new_pt);
    //      new_image_feature.laplacian_ = image_feature.laplacian_;
    //      new_image_feature.signature_ = image_feature.signature_;
    //      new_image_feature.n_observations_ = correspondence.n_observations_++;
    //      grid_map.update(correspondence, new_image_feature);
    //    }
    //    else
    //    {
    ImageFeature new_image_feature(image_feature.u_, image_feature.v_, image_feature.r_, image_feature.g_,
                                   image_feature.b_, m_pt);
    new_image_feature.laplacian_ = image_feature.laplacian_;
    new_image_feature.signature_ = image_feature.signature_;
    grid_map.insert(new_image_feature);
    //    }
  }
}


void VisualMapper::pixel2base(const Point& in_pt, Point& out_pt) const
{
  // Compute camera-world axis transformation matrix
  Pose cam2world(0., 0., 0, -M_PI / 2., 0., -M_PI / 2.);
  std::array<float, 9> c2w_rot{};
  cam2world.toRotMatrix(c2w_rot);
  Tf cam2world_tf(c2w_rot, std::array<float, 3>{ 0., 0., 0. });

  // Align world and camera axis
  Point wpoint = in_pt * cam2world_tf;

  // Compute camera-to-base transformation matrix
  Pose cam2base(cam2base_x_, cam2base_y_, cam2base_z_, cam2base_roll_, cam2base_pitch_, cam2base_yaw_);

  std::array<float, 9> c2b_rot{};
  cam2base.toRotMatrix(c2b_rot);
  Tf cam2base_tf(c2b_rot, std::array<float, 3>{ cam2base_x_, cam2base_y_, cam2base_z_ });

  // Transform camera point to base_link
  out_pt = wpoint * cam2base_tf.inverse();
}

}  // namespace vineslam
