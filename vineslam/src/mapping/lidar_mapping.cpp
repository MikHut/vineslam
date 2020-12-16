#include "../../include/vineslam/mapping/lidar_mapping.hpp"

namespace vineslam
{
LidarMapper::LidarMapper(const Parameters& params)
{
  // Set velodyne configuration parameters
  picked_num_ = 2;
  planes_th_ = static_cast<float>(60.) * DEGREE_TO_RAD;
  ground_th_ = static_cast<float>(5.) * DEGREE_TO_RAD;
  edge_threshold_ = 0.1;
  planar_threshold_ = 0.1;
  vertical_scans_ = 16;
  horizontal_scans_ = 1800;
  ground_scan_idx_ = 7;
  segment_valid_point_num_ = 5;
  segment_valid_line_num_ = 3;
  vertical_angle_bottom_ = static_cast<float>(15. + 0.1) * DEGREE_TO_RAD;
  ang_res_x_ = static_cast<float>(0.2) * DEGREE_TO_RAD;
  ang_res_y_ = static_cast<float>(2.) * DEGREE_TO_RAD;
  lidar_height = 1.20;

  // Initialize local map for downsampling
  Parameters local_map_params;
  local_map_params.gridmap_origin_x_ = -30;
  local_map_params.gridmap_origin_y_ = -30;
  local_map_params.gridmap_origin_z_ = -0.5;
  local_map_params.gridmap_resolution_ = 0.20;
  local_map_params.gridmap_width_ = 60;
  local_map_params.gridmap_lenght_ = 60;
  local_map_params.gridmap_height_ = 2.5;

  local_map_ = new OccupancyMap(local_map_params, Pose(0, 0, 0, 0, 0, 0));
}

void LidarMapper::registerMaps(const Pose& robot_pose, std::vector<Corner>& corners, std::vector<Planar>& planars,
                               std::vector<SemiPlane>& planes, OccupancyMap& grid_map)
{
  // - 3D PCL corner map estimation
  globalCornerMap(robot_pose, corners, grid_map);
  // - 3D PCL planar map estimation
  globalPlanarMap(robot_pose, planars, grid_map);
  // - 3D PCL plane map estimation
  globalPlaneMap(robot_pose, planes, grid_map);
}

// -------------------------------------------------------------------------------
// ---- 3D pointcloud feature map functions
// -------------------------------------------------------------------------------

void LidarMapper::reset()
{
  range_mat_.resize(vertical_scans_, horizontal_scans_);
  ground_mat_.resize(vertical_scans_, horizontal_scans_);
  label_mat_.resize(vertical_scans_, horizontal_scans_);
  range_mat_.fill(-1);
  ground_mat_.setZero();
  label_mat_.setZero();

  int cloud_size = vertical_scans_ * horizontal_scans_;

  seg_pcl_.start_col_idx.assign(vertical_scans_, 0);
  seg_pcl_.end_col_idx.assign(vertical_scans_, 0);
  seg_pcl_.is_ground.assign(cloud_size, false);
  seg_pcl_.col_idx.assign(cloud_size, 0);
  seg_pcl_.range.assign(cloud_size, 0);
}

void LidarMapper::localMap(const std::vector<Point>& pcl, std::vector<Corner>& out_corners,
                           std::vector<Planar>& out_planars, std::vector<SemiPlane>& out_planes,
                           SemiPlane& out_groundplane)
{
  // Build velodyne to base_link transformation matrix
  Pose tf_pose(vel2base_x_, vel2base_y_, vel2base_z_, vel2base_roll_, vel2base_pitch_, vel2base_yaw_);
  Tf tf;
  std::array<float, 9> tf_rot{};
  tf_pose.toRotMatrix(tf_rot);
  tf = Tf(tf_rot, std::array<float, 3>{ tf_pose.x_, tf_pose.y_, tf_pose.z_ });

  // Reset global variables and members
  reset();

  // Range image projection
  const size_t cloud_size = pcl.size();
  std::vector<Point> transformed_pcl(vertical_scans_ * horizontal_scans_);
  for (size_t i = 0; i < cloud_size; ++i)
  {
    Point l_pt = pcl[i];

    float range = l_pt.norm3D();

    // find the row and column index in the image for this point
    float vertical_angle = std::atan2(l_pt.z_, std::sqrt(l_pt.x_ * l_pt.x_ + l_pt.y_ * l_pt.y_));

    int row_idx = static_cast<int>((vertical_angle + vertical_angle_bottom_) / ang_res_y_);
    if (row_idx < 0 || row_idx >= vertical_scans_)
    {
      continue;
    }

    float horizon_angle = std::atan2(l_pt.x_, l_pt.y_);  // this is not an error

    int column_idx = static_cast<int>(-round((horizon_angle - M_PI_2) / ang_res_x_) + horizontal_scans_ / 2.);

    if (column_idx >= horizontal_scans_)
    {
      column_idx -= horizontal_scans_;
    }

    if (column_idx < 0 || column_idx >= horizontal_scans_)
    {
      continue;
    }

    if (range < 1.0 && range > 25.0)
    {
      continue;
    }

    range_mat_(row_idx, column_idx) = range;

    size_t idx = column_idx + row_idx * horizontal_scans_;
    transformed_pcl[idx] = l_pt;
  }

  // - Ground plane processing
  Plane unfiltered_gplane, filtered_gplane;
  // A - Extraction
  groundRemoval(transformed_pcl, unfiltered_gplane);
  // B - Filtering
  ransac(unfiltered_gplane.points_, filtered_gplane);
  // C - Centroid calculation
  for (const auto& pt : filtered_gplane.points_)
  {
    filtered_gplane.centroid_ = filtered_gplane.centroid_ + pt;
  }
  filtered_gplane.centroid_ = filtered_gplane.centroid_ / static_cast<float>(filtered_gplane.points_.size());
  // D - Bounding polygon
  convexHull(filtered_gplane, out_groundplane);
  for (auto& pt : out_groundplane.points_)
  {
    pt = pt * tf.inverse();
  }
  for (auto& pt : out_groundplane.extremas_)
  {
    pt = pt * tf.inverse();
  }
  out_groundplane.centroid_ = out_groundplane.centroid_ * tf.inverse();

  // -------------------------------------------------------------------------------
  // ----- Mark raw ground points
  // -------------------------------------------------------------------------------
  for (const auto& index : out_groundplane.indexes_)
  {
    int i = static_cast<int>(index.x_);
    int j = static_cast<int>(index.y_);

    ground_mat_(i, j) = 1;
    label_mat_(i, j) = -1;
  }

  // - Planes that are not the ground
  std::vector<PlanePoint> cloud_seg, cloud_seg_pure;
  cloudSegmentation(transformed_pcl, cloud_seg, cloud_seg_pure);

  // - Extract high level planes, and then convert them to semi-planes
  extractHighLevelPlanes(cloud_seg, out_planes);

  //- Corners feature extraction
  extractFeatures(cloud_seg, out_corners, out_planars);

  // - Convert local maps to base link
  for (auto& corner : out_corners)
  {
    corner.pos_ = corner.pos_ * tf.inverse();
  }
  for (auto& planar : out_planars)
  {
    planar.pos_ = planar.pos_ * tf.inverse();
  }
  for (auto& plane : out_planes)
  {
    for (auto& pt : plane.points_)
    {
      pt = pt * tf.inverse();
    }
    for (auto& pt : plane.extremas_)
    {
      pt = pt * tf.inverse();
    }
    plane.centroid_ = plane.centroid_ * tf.inverse();
  }

  // Downsample planar features
  local_map_->clear();
  for (const auto& planar : out_planars)
    local_map_->insert(planar);
  local_map_->downsamplePlanars();

  out_planars.clear();
  out_planars = local_map_->getPlanars();
}

void LidarMapper::globalCornerMap(const Pose& robot_pose, std::vector<Corner>& corners, OccupancyMap& grid_map)
{
  // ----------------------------------------------------------------------------
  // ------ Convert robot pose into homogeneous transformation
  // ----------------------------------------------------------------------------
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // Local array to store the new planar features
  std::vector<Corner> new_corners;

  // ----------------------------------------------------------------------------
  // ------ Insert corner into the grid map
  // ----------------------------------------------------------------------------
  for (auto& corner : corners)
  {
    // - First convert them to map's referential using the robot pose
    Point l_pt = corner.pos_ * tf;

    // - Then, look for correspondences in the local map
    Corner correspondence{};
    float best_correspondence = 0.02;
    bool found = false;
    std::vector<Corner> l_corners = grid_map(l_pt.x_, l_pt.y_, l_pt.z_).corner_features_;
    for (const auto& l_corner : l_corners)
    {
      float dist_min = l_pt.distance(l_corner.pos_);

      if (dist_min < best_correspondence)
      {
        correspondence = l_corner;
        best_correspondence = dist_min;
        found = true;
      }
    }

    found &= (best_correspondence < 0.02);

    // - Then, insert the corner into the grid map
    if (found)
    {
      Point new_pt = ((correspondence.pos_ * static_cast<float>(correspondence.n_observations_)) + l_pt) /
                     static_cast<float>(correspondence.n_observations_ + 1);
      Corner new_corner(new_pt, corner.which_plane_);
      new_corner.n_observations_ = correspondence.n_observations_ + 1;
      grid_map.update(correspondence, new_corner);
    }
    else
    {
      Corner new_corner(l_pt, corner.which_plane_);
      new_corners.push_back(new_corner);
    }
  }

  // Insert the new observations found
  for (const auto& corner : new_corners)
    grid_map.insert(corner);
}

void LidarMapper::globalPlanarMap(const Pose& robot_pose, std::vector<Planar>& planars, OccupancyMap& grid_map)
{
  // ----------------------------------------------------------------------------
  // ------ Convert robot pose into homogeneous transformation
  // ----------------------------------------------------------------------------
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // Local array to store the new planar features
  std::vector<Planar> new_planars;

  // ----------------------------------------------------------------------------
  // ------ Insert planar into the grid map
  // ----------------------------------------------------------------------------
  for (auto& planar : planars)
  {
    // - First convert them to map's referential using the robot pose
    Point l_pt = planar.pos_ * tf;

    // - Then, look for correspondences in the local map
    Planar correspondence{};
    float best_correspondence = 0.02;
    bool found = false;
    std::vector<Planar> l_planars = grid_map(l_pt.x_, l_pt.y_, l_pt.z_).planar_features_;
    for (const auto& l_planar : l_planars)
    {
      float dist_min = l_pt.distance(l_planar.pos_);

      if (dist_min < best_correspondence)
      {
        correspondence = l_planar;
        best_correspondence = dist_min;
        found = true;
      }
    }

    found &= (best_correspondence < 0.02);

    // - Then, insert the planar into the grid map
    if (found)
    {
      Point new_pt = ((correspondence.pos_ * static_cast<float>(correspondence.n_observations_)) + l_pt) /
                     static_cast<float>(correspondence.n_observations_ + 1);
      Planar new_planar(new_pt, planar.which_plane_);
      new_planar.n_observations_ = correspondence.n_observations_ + 1;
      grid_map.update(correspondence, new_planar);
    }
    else
    {
      Planar new_planar(l_pt, planar.which_plane_);
      new_planars.push_back(new_planar);
    }
  }

  // Insert the new observations found
  for (const auto& planar : new_planars)
    grid_map.insert(planar);
}

void LidarMapper::globalPlaneMap(const Pose& robot_pose, std::vector<SemiPlane>& planes, OccupancyMap& grid_map)
{
  // ----------------------------------------------------------------------------
  // ------ Search for correspondences between local planes and global planes
  // ------ Two stage process:
  // ------  * (A) Check semi-plane overlap
  // ------  * (B) Compare planes normals
  // ------  *  If (B), then check (C) plane to plane distance
  // ----------------------------------------------------------------------------

  // Convert robot pose into homogeneous transformation
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // Define correspondence thresholds
  float th_dist = 15 * DEGREE_TO_RAD;  // max normal angular distance per component
  float sp_dist = 0.1;                 // max distance from source plane centroid to target plane
  float area_th = 2.0;                 // minimum overlapping area between semiplanes

  // Array to store the new planes observed
  std::vector<SemiPlane> new_planes;

  for (const auto& plane : planes)
  {
    if (plane.points_.empty())
    {
      continue;
    }

    // Initialize correspondence deltas
    Pose delta_rot(0, 0, 0, th_dist, th_dist, th_dist);
    float point2plane = sp_dist;
    float ov_area = area_th;

    // Declare plane to store the correspondence
    SemiPlane* correspondence;

    // Convert local plane to maps' referential frame
    SemiPlane l_plane = plane;
    for (auto& point : l_plane.points_)
    {
      point = point * tf;  // Convert plane points
    }
    for (auto& point : l_plane.extremas_)
    {
      point = point * tf;  // Convert plane boundaries
    }
    l_plane.centroid_ = l_plane.centroid_ * tf;                                       // Convert the centroid
    estimateNormal(l_plane.points_, l_plane.a_, l_plane.b_, l_plane.c_, l_plane.d_);  // Convert plane normal
    l_plane.setLocalRefFrame();

    std::cout << "Started loop .... \n";
    bool found = false;
    for (auto& g_plane : grid_map.planes_)
    {
      if (g_plane.points_.empty())
      {
        continue;
      }

      // --------------------------------
      // (A) - Check semi-plane overlap
      // --------------------------------

      // First project the global and local plane extremas to the global plane reference frame
      Tf ref_frame = g_plane.local_ref_.inverse();
      SemiPlane gg_plane;
      SemiPlane lg_plane;
      for (const auto& extrema : g_plane.extremas_)
      {
        Point p = extrema * ref_frame;
        p.z_ = 0;
        gg_plane.extremas_.push_back(p);
      }
      for (const auto& extrema : l_plane.extremas_)
      {
        Point p = extrema * ref_frame;
        p.z_ = 0;
        lg_plane.extremas_.push_back(p);
      }

      // Now, check for transformed polygon intersections
      SemiPlane isct;
      polygonIntersection(gg_plane, lg_plane, isct.extremas_);

      // Compute the intersection semi plane area
      isct.setArea();

      if (isct.area_ > ov_area)
      {
        // --------------------------------
        // (B) - Compare plane normals
        // --------------------------------

        // Find the rotation matrix that transforms one normal vector into another
        Vec u(l_plane.a_, l_plane.b_, l_plane.c_);
        Vec v(g_plane.a_, g_plane.b_, g_plane.c_);

        std::array<float, 9> R = u.rotation(v);

        // Extract the euler angles from the rotation matrix
        Pose l_delta_rot = Pose(R, std::array<float, 3>{ 0, 0, 0 });

        // Account for parallel vectors in opposite directions
        l_delta_rot.R_ = std::min(std::fabs(l_delta_rot.R_), static_cast<float>(M_PI) - std::fabs(l_delta_rot.R_));
        l_delta_rot.P_ = std::min(std::fabs(l_delta_rot.P_), static_cast<float>(M_PI) - std::fabs(l_delta_rot.P_));
        l_delta_rot.Y_ = std::min(std::fabs(l_delta_rot.Y_), static_cast<float>(M_PI) - std::fabs(l_delta_rot.Y_));
        std::cout << "Passed area normal is " << l_delta_rot.R_ * RAD_TO_DEGREE << ", "
                  << l_delta_rot.P_ * RAD_TO_DEGREE << ", " << l_delta_rot.Y_ * RAD_TO_DEGREE << "\n";

        // Check if normal vectors match
        if (std::fabs(l_delta_rot.R_) < std::fabs(delta_rot.R_) &&
            std::fabs(l_delta_rot.P_) < std::fabs(delta_rot.P_) && std::fabs(l_delta_rot.Y_) < std::fabs(delta_rot.Y_))
        {
          // --------------------------------
          // (C) - Compute local plane centroid distance to global plane
          // --------------------------------
          float l_point2plane = g_plane.point2Plane(l_plane.centroid_);
          //          std::cout << "Passed area and normal, centroid is " << l_point2plane << "\n";
          if (l_point2plane < point2plane)
          {
            // We found a correspondence, so, we must save the correspondence deltas
            delta_rot = l_delta_rot;
            point2plane = l_point2plane;
            ov_area = isct.area_;

            std::cout << "FOUND CORRESPONDENCE: area = " << ov_area << ", centroid = " << point2plane
                      << ", rot = " << l_delta_rot << "\n";

            // Save the correspondence semi-plane
            correspondence = &g_plane;

            // Set correspondence flag
            found = true;
          }
        }
      }
    }

    // Check if a correspondence was found
    if (found)
    {
      // If so, update plane on global map with new observation (registration)

      // Insert the new observation points into the correspondent global map plane
      correspondence->points_.insert(correspondence->points_.end(), l_plane.points_.begin(), l_plane.points_.end());

      // Re-compute centroid
      correspondence->centroid_ = Point(0, 0, 0);
      for (const auto& pt : correspondence->points_)
      {
        correspondence->centroid_ = correspondence->centroid_ + pt;
      }
      correspondence->centroid_ = correspondence->centroid_ / static_cast<float>(correspondence->points_.size());

      // Re-compute the plane normal
      float l_a, l_b, l_c, l_d;
      estimateNormal(correspondence->points_, l_a, l_b, l_c, l_d);
      correspondence->a_ = l_a;
      correspondence->b_ = l_b;
      correspondence->c_ = l_c;
      correspondence->d_ = l_d;
      correspondence->setLocalRefFrame();

      // Re-compute the semi plane boundaries
      Plane filter_plane(l_a, l_b, l_c, l_d, correspondence->points_);
      SemiPlane filter_semiplane;
      convexHull(filter_plane, filter_semiplane);
      correspondence->extremas_ = filter_semiplane.extremas_;
      correspondence->setArea();
    }
    else
    {
      // If not, add a new plane to the map
      new_planes.push_back(l_plane);
    }
  }

  // Add new planes found to the map
  if (!new_planes.empty())
  {
    grid_map.planes_.insert(grid_map.planes_.end(), new_planes.begin(), new_planes.end());
  }
}

void LidarMapper::groundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl)
{
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (int j = 0; j < horizontal_scans_; j++)
  {
    for (int i = 0; i < ground_scan_idx_; i++)
    {
      int lower_idx = j + i * horizontal_scans_;
      int upper_idx = j + (i + 1) * horizontal_scans_;

      Point upper_pt = in_pts[upper_idx];
      Point lower_pt = in_pts[lower_idx];

      if (range_mat_(i, j) == -1 || range_mat_(i + 1, j) == -1)
      {
        // no info to check, invalid points
        ground_mat_(i, j) = -1;
        continue;
      }

      float dX = upper_pt.x_ - lower_pt.x_;
      float dY = upper_pt.y_ - lower_pt.y_;
      float dZ = upper_pt.z_ - lower_pt.z_;

      float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

      if (vertical_angle <= ground_th_ && std::fabs(lower_pt.z_) > lidar_height / 2 &&
          std::fabs(upper_pt.z_) > lidar_height / 2)
      {
        out_pcl.points_.push_back(lower_pt);
        out_pcl.points_.push_back(upper_pt);
        out_pcl.indexes_.emplace_back(i, j);
        out_pcl.indexes_.emplace_back(i + 1, j);
      }
    }
  }
}

void LidarMapper::cloudSegmentation(const std::vector<Point>& in_pts, std::vector<PlanePoint>& cloud_seg,
                                    std::vector<PlanePoint>& cloud_seg_pure)
{
  // Segmentation process
  int label = 1;
  for (int i = 0; i < vertical_scans_; i++)
  {
    for (int j = 0; j < horizontal_scans_; j++)
    {
      if (label_mat_(i, j) == 0 && range_mat_(i, j) != -1)
        labelComponents(i, j, in_pts, label);
    }
  }

  // Extract segmented cloud for visualization
  int seg_cloud_size = 0;
  for (int i = 0; i < vertical_scans_; i++)
  {
    seg_pcl_.start_col_idx[i] = seg_cloud_size - 1 + 5;

    for (int j = 0; j < horizontal_scans_; j++)
    {
      if (label_mat_(i, j) > 0 || ground_mat_(i, j) == 1)
      {
        if (label_mat_(i, j) == 999999)
          continue;

        // The majority of ground points are skipped
        if (ground_mat_(i, j) == 1)
        {
          //          if (j % 5 != 0 && j > 5 && j < horizontal_scans_ - 5)
          //          {
          continue;
          //          }
        }

        // Mark ground points so they will not be considered as edge features later
        seg_pcl_.is_ground[seg_cloud_size] = (ground_mat_(i, j) == 1);

        // Save segmented cloud into a pcl
        Point pt = in_pts[j + i * horizontal_scans_];
        PlanePoint plane_pt(pt, label_mat_(i, j));
        cloud_seg.push_back(plane_pt);
        // ------------------------------------------
        // Save segmented cloud in the given structure
        seg_pcl_.col_idx[seg_cloud_size] = j;
        seg_pcl_.range[seg_cloud_size] = range_mat_(i, j);
        seg_cloud_size++;
        // ------------------------------------------
      }
    }
    seg_pcl_.end_col_idx[i] = seg_cloud_size - 1 - 5;
  }
}

void LidarMapper::labelComponents(const int& row, const int& col, const std::vector<Point>& in_pts, int& label)
{
  using Coord2D = Eigen::Vector2i;
  std::deque<Coord2D> queue;
  std::deque<Coord2D> global_queue;

  queue.emplace_back(row, col);
  global_queue.emplace_back(row, col);

  std::vector<bool> line_count_flag(vertical_scans_, false);

  // - Define neighborhood
  const Coord2D neighbor_it[4] = { { 0, -1 }, { -1, 0 }, { 1, 0 }, { 0, 1 } };

  while (!queue.empty())
  {
    // Evaluate front element of the queue and pop it
    Coord2D from_idx = queue.front();
    queue.pop_front();

    // Mark popped point as belonging to the segment
    label_mat_(from_idx.x(), from_idx.y()) = label;

    // Compute point from range image
    float d1 = range_mat_(from_idx.x(), from_idx.y());

    // Loop through all the neighboring grids of popped grid
    for (const auto& iter : neighbor_it)
    {
      // Compute new index
      int c_idx_x = from_idx.x() + iter.x();
      int c_idx_y = from_idx.y() + iter.y();

      // Check if index is within the boundary
      if (c_idx_x < 0 || c_idx_x >= vertical_scans_)
        continue;
      if (c_idx_y < 0)
        c_idx_y = horizontal_scans_ - 1;
      if (c_idx_y >= horizontal_scans_)
        c_idx_y = 0;

      // Prevent infinite loop (caused by put already examined point back)
      if (label_mat_(c_idx_x, c_idx_y) != 0)
        continue;

      // Compute point from range image
      float d2 = range_mat_(c_idx_x, c_idx_y);

      float dmax = std::max(d1, d2);
      float dmin = std::min(d1, d2);

      // Compute angle between the two points
      float alpha = (iter.x() == 0) ? ang_res_x_ : ang_res_y_;

      // Compute beta and check if points belong to the same segment
      auto beta = std::atan2((dmin * std::sin(alpha)), (dmax - dmin * std::cos(alpha)));
      if (beta > planes_th_)
      {
        queue.emplace_back(c_idx_x, c_idx_y);
        global_queue.emplace_back(c_idx_x, c_idx_y);

        label_mat_(c_idx_x, c_idx_y) = label;
        line_count_flag[c_idx_x] = true;
      }
    }
  }

  // Check if this segment is valid
  bool feasible_segment = false;
  if (global_queue.size() >= 30)
  {
    feasible_segment = true;
  }
  else if (global_queue.size() >= segment_valid_point_num_)
  {
    int line_count = 0;
    for (int i = 0; i < vertical_scans_; i++)
    {
      if (line_count_flag[i])
        line_count++;
    }

    if (line_count >= segment_valid_line_num_)
      feasible_segment = true;
  }

  if (feasible_segment)
  {
    label++;
  }
  else
  {
    for (auto& i : global_queue)
      label_mat_(i.x(), i.y()) = 999999;
  }
}

void LidarMapper::extractHighLevelPlanes(const std::vector<PlanePoint>& in_plane_pts,
                                         std::vector<SemiPlane>& out_planes)
{
  // -------------------------------------------------------------------------------
  // ----- Segment plane points in two different sets
  // -------------------------------------------------------------------------------
  // - Start by computing the average of the y component of all points
  float y_mean = 0.;
  for (auto const& plane_pt : in_plane_pts)
    y_mean += plane_pt.pos_.y_;
  y_mean /= static_cast<float>(in_plane_pts.size());

  // - Cluster points using the y mean threshold
  Plane side_plane_a, side_plane_b;
  for (auto const& plane_pt : in_plane_pts)
  {
    PlanePoint m_plane_pt = plane_pt;

    m_plane_pt.which_plane_ = (m_plane_pt.pos_.y_ < y_mean) ? 0 : 1;

    if (m_plane_pt.which_plane_ == 0)
    {
      side_plane_a.points_.push_back(m_plane_pt.pos_);
    }
    else
    {
      side_plane_b.points_.push_back(m_plane_pt.pos_);
    }
  }

  // - Remove outliers using RANSAC
  Plane side_plane_a_filtered, side_plane_b_filtered;
  ransac(side_plane_a.points_, side_plane_a_filtered, 100, 0.05);
  ransac(side_plane_b.points_, side_plane_b_filtered, 100, 0.05);
  side_plane_a_filtered.id_ = 0;
  side_plane_b_filtered.id_ = 1;

  // -------------------------------------------------------------------------------
  // ----- Check the validity of the extracted planes
  // -------------------------------------------------------------------------------
  std::vector<Plane> planes = { side_plane_a_filtered, side_plane_b_filtered };

  for (auto& plane : planes)
  {
    // Check if the plane have a minimum number of points
    plane.centroid_ = Point(0, 0, 0);
    for (const auto& pt : plane.points_)
    {
      plane.centroid_ = plane.centroid_ + pt;
    }
    plane.centroid_ = plane.centroid_ / static_cast<float>(plane.points_.size());
    plane.setLocalRefFrame();

    SemiPlane l_semi_plane;
    if (convexHull(plane, l_semi_plane) && l_semi_plane.area_ > 2 && plane.points_.size() > 500)
    {
      out_planes.push_back(l_semi_plane);
    }
  }
}

void LidarMapper::extractFeatures(const std::vector<PlanePoint>& in_plane_pts, std::vector<Corner>& out_corners,
                                  std::vector<Planar>& out_planars)

{
  // -------------------------------------------------------------------------------
  // ----- Compute cloud smoothness
  // -------------------------------------------------------------------------------
  int* cloudPlanarLabel = new int[vertical_scans_ * horizontal_scans_];
  int* cloudCornerLabel = new int[vertical_scans_ * horizontal_scans_];
  int l_cloud_size = in_plane_pts.size();
  std::vector<smoothness_t> cloud_smoothness(vertical_scans_ * horizontal_scans_);
  std::vector<int> neighbor_picked(vertical_scans_ * horizontal_scans_);
  for (int i = 5; i < l_cloud_size - 5; i++)
  {
    // Compute smoothness and save it
    float diff_range = seg_pcl_.range[i - 5] + seg_pcl_.range[i - 4] + seg_pcl_.range[i - 3] + seg_pcl_.range[i - 2] +
                       seg_pcl_.range[i - 1] + seg_pcl_.range[i + 1] + seg_pcl_.range[i + 2] + seg_pcl_.range[i + 3] +
                       seg_pcl_.range[i + 4] + seg_pcl_.range[i + 5] - 10 * seg_pcl_.range[i];

    cloud_smoothness[i].value = diff_range * diff_range;
    cloud_smoothness[i].idx = i;

    cloudPlanarLabel[i] = 0;
    cloudCornerLabel[i] = 0;

    // Reset neighborhood flag array
    neighbor_picked[i] = 0;
  }

  // -------------------------------------------------------------------------------
  // ----- Mark occluded points
  // -------------------------------------------------------------------------------

  for (int i = 5; i < l_cloud_size - 6; ++i)
  {
    float depth1 = seg_pcl_.range[i];
    float depth2 = seg_pcl_.range[i + 1];
    int col_diff = std::abs(int(seg_pcl_.col_idx[i + 1] - seg_pcl_.col_idx[i]));

    if (col_diff < 10)
    {
      if (depth1 - depth2 > 0.3)
      {
        neighbor_picked[i - 5] = 1;
        neighbor_picked[i - 4] = 1;
        neighbor_picked[i - 3] = 1;
        neighbor_picked[i - 2] = 1;
        neighbor_picked[i - 1] = 1;
        neighbor_picked[i] = 1;
      }
      else if (depth2 - depth1 > 0.3)
      {
        neighbor_picked[i + 1] = 1;
        neighbor_picked[i + 2] = 1;
        neighbor_picked[i + 3] = 1;
        neighbor_picked[i + 4] = 1;
        neighbor_picked[i + 5] = 1;
        neighbor_picked[i + 6] = 1;
      }
    }

    float diff1 = std::abs(float(seg_pcl_.range[i - 1] - seg_pcl_.range[i]));
    float diff2 = std::abs(float(seg_pcl_.range[i + 1] - seg_pcl_.range[i]));

    if (diff1 > 0.02 * seg_pcl_.range[i] && diff2 > 0.02 * seg_pcl_.range[i])
      neighbor_picked[i] = 1;
  }

  // -------------------------------------------------------------------------------
  // ----- Extract features from the 3D cloud
  // -------------------------------------------------------------------------------
  std::vector<Planar> planar_points_less_flat;
  int corner_id = 0;
  int planar_id = 0;
  for (int i = 0; i < vertical_scans_; i++)
  {
    planar_points_less_flat.clear();

    for (int k = 0; k < 6; k++)
    {
      // Compute start and end indexes of the sub-region
      int sp = (seg_pcl_.start_col_idx[i] * (6 - k) + (seg_pcl_.end_col_idx[i] * k)) / 6;
      int ep = (seg_pcl_.start_col_idx[i] * (5 - k) + (seg_pcl_.end_col_idx[i] * (k + 1))) / 6 - 1;

      if (sp >= ep)
        continue;

      // Sort smoothness values for the current sub-region
      std::sort(cloud_smoothness.begin() + sp, cloud_smoothness.begin() + ep, by_value());

      // -- Extract edge features
      int picked_counter = 0;
      for (int l = ep; l >= sp; l--)
      {
        int idx = cloud_smoothness[l].idx;

        // Check if the current point is an edge feature
        if (neighbor_picked[idx] == 0 && cloud_smoothness[l].value > edge_threshold_ && !seg_pcl_.is_ground[idx])
        {
          picked_counter++;
          if (picked_counter <= picked_num_)
          {
            Corner l_corner(in_plane_pts[idx].pos_, in_plane_pts[idx].which_plane_, corner_id);
            out_corners.push_back(l_corner);
            corner_id++;
          }
          else
          {
            break;
          }

          cloudCornerLabel[idx] = -1;

          // Mark neighbor points to reject as future features
          neighbor_picked[idx] = 1;
          for (int m = 1; m <= 5; m++)
          {
            if (idx + m >= seg_pcl_.col_idx.size())
              continue;
            int col_diff = std::abs(seg_pcl_.col_idx[idx + m] - seg_pcl_.col_idx[idx + m - 1]);
            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
          for (int m = -1; m >= -5; m--)
          {
            if (idx + m < 0)
              continue;
            int col_diff = std::abs(seg_pcl_.col_idx[idx + m] - seg_pcl_.col_idx[idx + m + 1]);
            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
        }
      }

      // -- Extract planar features
      picked_counter = 0;
      for (int l = sp; l <= ep; l++)
      {
        int idx = cloud_smoothness[l].idx;

        // Check if the current point is a planar feature
        if (neighbor_picked[idx] == 0 && cloud_smoothness[l].value < planar_threshold_)
        {
          cloudPlanarLabel[idx] = -1;

          picked_counter++;
          if (picked_counter >= 4)
            break;

          neighbor_picked[idx] = 1;
          for (int m = 1; m <= 5; m++)
          {
            int col_diff = std::abs(seg_pcl_.col_idx[idx + m] - seg_pcl_.col_idx[idx + m - 1]);

            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
          for (int m = -1; m >= -5; m--)
          {
            if (idx + m < 0)
              continue;
            int col_diff = std::abs(seg_pcl_.col_idx[idx + m] - seg_pcl_.col_idx[idx + m + 1]);
            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
        }
      }

      for (int l = sp; l <= ep; l++)
      {
        if (cloudPlanarLabel[l] <= 0 && cloudCornerLabel[l] >= 0)
        {
          Planar planar(in_plane_pts[l].pos_, in_plane_pts[l].which_plane_, planar_id);
          planar_points_less_flat.push_back(planar);
        }
      }
    }

    out_planars.insert(out_planars.end(), planar_points_less_flat.begin(), planar_points_less_flat.end());
  }
}

}  // namespace vineslam