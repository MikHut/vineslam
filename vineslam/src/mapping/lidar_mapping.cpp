#include "../../include/vineslam/mapping/lidar_mapping.hpp"

namespace vineslam
{
LidarMapper::LidarMapper()
{
  filter_frequency_ = 30;
  it_ = 0;
}

void LidarMapper::registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners,
                               const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes,
                               const SemiPlane& ground, OccupancyMap& grid_map, ElevationMap& elevation_map)
{
  // - 3D PCL corner map estimation
  globalCornerMap(robot_pose, corners, grid_map);
  // - 3D PCL planar map estimation
  globalPlanarMap(robot_pose, planars, grid_map);
  // - 3D PCL plane map estimation
  globalPlaneMap(robot_pose, planes, grid_map);
  globalPlaneMap(robot_pose, { ground }, grid_map);
  // - Elevation map estimation
  globalElevationMap(robot_pose, ground, elevation_map);

  // Store robot pose to use in the next iteration
  prev_robot_pose_ = robot_pose;

  // Increment mapper iterator
  it_++;
}

void LidarMapper::registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners,
                               const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes,
                               const SemiPlane& ground, OccupancyMap& grid_map)
{
  // - 3D PCL corner map estimation
  globalCornerMap(robot_pose, corners, grid_map);
  // - 3D PCL planar map estimation
  globalPlanarMap(robot_pose, planars, grid_map);
  // - 3D PCL plane map estimation
  globalPlaneMap(robot_pose, planes, grid_map);
  globalPlaneMap(robot_pose, { ground }, grid_map);

  // Store robot pose to use in the next iteration
  prev_robot_pose_ = robot_pose;

  // Increment mapper iterator
  it_++;
}

void LidarMapper::globalCornerMap(const Pose& robot_pose, const std::vector<Corner>& corners, OccupancyMap& grid_map)
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
    float best_correspondence = 0.20;
    bool found = false;
    std::vector<Corner>* l_corners{ nullptr };

    if (grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data != nullptr)
    {
      l_corners = grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->corner_features_;
    }

    if (l_corners != nullptr)
    {
      for (const auto& l_corner : *l_corners)
      {
        float dist_min = l_pt.distance(l_corner.pos_);

        if (dist_min < best_correspondence)
        {
          correspondence = l_corner;
          best_correspondence = dist_min;
          found = true;
        }
      }

      found &= (best_correspondence < 0.2);
    }

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

void LidarMapper::globalPlanarMap(const Pose& robot_pose, const std::vector<Planar>& planars, OccupancyMap& grid_map)
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
    float best_correspondence = 0.20;
    bool found = false;
    std::vector<Planar>* l_planars = { nullptr };

    if (grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data != nullptr)
    {
      l_planars = grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_;
    }

    if (l_planars != nullptr)
    {
      for (const auto& l_planar : *l_planars)
      {
        float dist_min = l_pt.distance(l_planar.pos_);

        if (dist_min < best_correspondence)
        {
          correspondence = l_planar;
          best_correspondence = dist_min;
          found = true;
        }
      }

      found &= (best_correspondence < 0.2);
    }

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

void LidarMapper::globalPlaneMap(const Pose& robot_pose, const std::vector<SemiPlane>& planes, OccupancyMap& grid_map)
{
  // ----------------------------------------------------------------------------
  // ------ First, we remove semiplane outliers in the global map checking their
  // ------        number of correspondences at a well defined frequency
  // ----------------------------------------------------------------------------
  if (it_ % filter_frequency_ == 0)
  {
    int planes_size = static_cast<int>(grid_map.planes_.size());
    for (int i = 0; i < planes_size; i++)
    {
      if (grid_map.planes_[i].n_occurences_ > filter_frequency_ && grid_map.planes_[i].n_correspondences_ < 10)
      {
        grid_map.planes_.erase(grid_map.planes_.begin() + i);
      }
    }
  }

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
  float v_dist = 0.2;   // max vector displacement for all the components
  float sp_dist = 0.3;  // max distance from source plane centroid to target plane
  float area_th = 1.0;  // minimum overlapping area between semiplanes

  // Array to store the new planes observed
  std::vector<SemiPlane> new_planes;

  for (const auto& plane : planes)
  {
    if (plane.points_.empty())
    {
      continue;
    }

    // Initialize correspondence deltas
    float vec_disp = v_dist;
    float point2plane = sp_dist;
    float ov_area = area_th;

    // Declare plane to store the correspondence
    auto* correspondence = new SemiPlane();

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
    l_plane.centroid_ = l_plane.centroid_ * tf;                                               // Convert the centroid
    Ransac::estimateNormal(l_plane.points_, l_plane.a_, l_plane.b_, l_plane.c_, l_plane.d_);  // Convert plane normal
    l_plane.setLocalRefFrame();

    bool found = false;
    for (auto& g_plane : grid_map.planes_)
    {
      // Increment the number of visits to the plane (used to filter planes with low correspondences)
      g_plane.n_occurences_++;

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
      ConvexHull::polygonIntersection(gg_plane, lg_plane, isct.extremas_);

      // Compute the intersection semi plane area
      isct.setArea();

      if (isct.area_ > ov_area)
      {
        // --------------------------------
        // (B) - Compare plane normals
        // --------------------------------

        Vec u(l_plane.a_, l_plane.b_, l_plane.c_);
        Vec v(g_plane.a_, g_plane.b_, g_plane.c_);

        float D = ((u - v).norm3D() < (u + v).norm3D()) ? (u - v).norm3D() : (u + v).norm3D();

        // Check if normal vectors match
        if (D < vec_disp)
        {
          // --------------------------------
          // (C) - Compute local plane centroid distance to global plane
          // --------------------------------
          float l_point2plane = g_plane.point2Plane(l_plane.centroid_);
          if (l_point2plane < point2plane)
          {
            // We found a correspondence, so, we must save the correspondence deltas
            vec_disp = D;
            point2plane = l_point2plane;
            ov_area = isct.area_;

            // Update number of correspondences
            g_plane.n_correspondences_++;

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
      Ransac::estimateNormal(correspondence->points_, l_a, l_b, l_c, l_d);
      correspondence->a_ = l_a;
      correspondence->b_ = l_b;
      correspondence->c_ = l_c;
      correspondence->d_ = l_d;
      correspondence->setLocalRefFrame();

      // Re-compute the semi plane boundaries
      Plane filter_plane(l_a, l_b, l_c, l_d, correspondence->points_);
      SemiPlane filter_semiplane;
      ConvexHull::process(filter_plane, filter_semiplane);
      correspondence->extremas_ = filter_semiplane.extremas_;
      correspondence->setArea();

      correspondence->points_ =
          correspondence->extremas_;  // This is a trick to improve performance: to update a semiplane, we only need the
                                      // previously calculated extremas and the newly observed points :)
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

void LidarMapper::globalElevationMap(const Pose& robot_pose, const Plane& ground_plane, ElevationMap& elevation_map)
{
  // ----------------------------------------------------------------------------
  // ------ Convert robot pose into homogeneous transformation
  // ----------------------------------------------------------------------------
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // ----------------------------------------------------------------------------
  // ------ Add new altemetry measures from ground plane to the elevation map
  // ----------------------------------------------------------------------------
  for (const auto& pt : ground_plane.points_)
  {
    Point l_pt = pt * tf;
    elevation_map.update(l_pt.z_, l_pt.x_, l_pt.y_);
  }

  // ----------------------------------------------------------------------------
  // ------ Add new altemetry measures from robot pose to the elevation map
  // ----------------------------------------------------------------------------

  // Discretize robot box using the elevation map resolution
  for (float i = 0; i < robot_dim_x_;)
  {
    for (float j = 0; j < robot_dim_y_;)
    {
      Point pt(i - robot_dim_x_ / 2, j - robot_dim_y_ / 2, 0);
      Point pt_transformed = pt * tf;

      elevation_map.update(pt_transformed.z_, pt_transformed.x_, pt_transformed.y_);

      j += elevation_map.resolution_;
    }
    i += elevation_map.resolution_;
  }
}

void LidarMapper::computeUnoccupiedZone(const std::vector<Point>& in_pts, std::vector<Point>& rectangle)
{
  // Compute an unoccupied rectangle around the robot
  // Creterias:
  //    - use only 50% of the minimum distance observed (works as an erosion)
  //    - limit the maximum size of the rectangle

  float right_min_y = std::numeric_limits<float>::max();
  float left_min_y = std::numeric_limits<float>::max();
  float front_min_x = std::numeric_limits<float>::max();
  float back_min_x = std::numeric_limits<float>::max();

  for (const auto& pt : in_pts)
  {
    if (std::fabs(pt.x_) < 1.5 || std::fabs(pt.y_) < 0.5)
    {
      continue;
    }

    if (pt.x_ < 0 && std::fabs(pt.x_) < back_min_x)
    {
      back_min_x = pt.x_;
    }
    if (pt.x_ > 0 && std::fabs(pt.x_) < front_min_x)
    {
      front_min_x = pt.x_;
    }
    if (pt.y_ < 0 && std::fabs(pt.y_) < left_min_y)
    {
      left_min_y = pt.y_;
    }
    if (pt.y_ > 0 && std::fabs(pt.y_) < right_min_y)
    {
      right_min_y = pt.y_;
    }
  }

  back_min_x = (std::fabs(back_min_x) < 6) ? (back_min_x * 0.5) : (-6 * 0.5);
  front_min_x = (std::fabs(front_min_x) < 6) ? (front_min_x * 0.5) : (6 * 0.5);
  left_min_y = (std::fabs(left_min_y) < 2) ? (left_min_y * 0.5) : (-2 * 0.5);
  right_min_y = (std::fabs(right_min_y) < 2) ? (right_min_y * 0.5) : (-2 * 0.5);

  // Create rectangle with the computed limit values
  Point left_upper(front_min_x, left_min_y);
  Point right_upper(front_min_x, right_min_y);
  Point left_bottom(back_min_x, left_min_y);
  Point right_bottom(back_min_x, right_min_y);

  // Save the rectangle
  rectangle.push_back(left_bottom);
  rectangle.push_back(right_upper);
}

void LidarMapper::filterWithinZone(const Pose& robot_pose, const std::vector<Point>& rectangle, OccupancyMap& grid_map)
{
  if (rectangle.size() != 2)
  {
    return;
  }

  // ----------------------------------------------------------------------------
  // ------ Convert robot pose into homogeneous transformation
  // ----------------------------------------------------------------------------
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // ----------------------------------------------------------------------------
  // ------ Access the grid map cells inside the rectangle and filter
  // ------ We consider that the rectangle is [left_bottom, right_upper]
  // ----------------------------------------------------------------------------
  float z_min = robot_pose.z_ + 0.2;
  float z_max = robot_pose.z_ + 1.9;
  for (float i = rectangle[0].x_; i <= rectangle[1].x_;)
  {
    for (float j = rectangle[0].y_; j <= rectangle[1].y_;)
    {
      Point pt(i + 0.49, j + 0.49, 0);
      Point projected_pt = pt * tf;

      for (float z = z_min; z <= z_max;)
      {
        Cell* c = &(grid_map)(projected_pt.x_, projected_pt.y_, z);
        if (c->data == nullptr)
        {
          z += grid_map.resolution_z_;
          continue;
        }
        std::vector<Planar>* l_planars = c->data->planar_features_;
        std::vector<Corner>* l_corners = c->data->corner_features_;
        if (l_corners != nullptr)
        {
          // Now we filter (!)
          // First, check if the point is close to the ground. If so, we do not remove it
          for (size_t l = 0; l < l_corners->size(); l++)
          {
            l_corners->erase(l_corners->begin() + l);
          }
        }
        if (l_planars != nullptr)
        {
          // Now we filter (!)
          // First, check if the point is close to the ground. If so, we do not remove it
          for (size_t l = 0; l < l_planars->size(); l++)
          {
            l_planars->erase(l_planars->begin() + l);
          }
        }

        z += grid_map.resolution_z_;
      }

      j += grid_map.resolution_;
    }
    i += grid_map.resolution_;
  }
}

// -------------------------------------------------------------------
// ----- Velodyne functions
// -------------------------------------------------------------------

VelodyneMapper::VelodyneMapper(const Parameters& params)
{
  // Set velodyne configuration parameters
  picked_num_ = 2;
  if (params.lightweight_version_ == true)
  {
    planes_th_ = static_cast<float>(85.) * DEGREE_TO_RAD;
  }
  else
  {
    planes_th_ = static_cast<float>(60) * DEGREE_TO_RAD;
  }
  ground_th_ = static_cast<float>(3.) * DEGREE_TO_RAD;
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
  lidar_height_ = 1.20;

  // Set robot dimensions for elevation map computation
  robot_dim_x_ = params.robot_dim_x_;
  robot_dim_y_ = params.robot_dim_y_;
  robot_dim_z_ = params.robot_dim_z_;

  // Set previous robot pose handler
  prev_robot_pose_ = Pose(0, 0, 0, 0, 0, 0);
}

void VelodyneMapper::flatGroundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl)
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
        //        ground_mat_(i, j) = -1;
        continue;
      }

      float dX = upper_pt.x_ - lower_pt.x_;
      float dY = upper_pt.y_ - lower_pt.y_;
      float dZ = upper_pt.z_ - lower_pt.z_;

      float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

      if (vertical_angle <= ground_th_ && std::fabs(lower_pt.z_) > lidar_height_ / 2 &&
          std::fabs(upper_pt.z_) > lidar_height_ / 2)
      {
        out_pcl.points_.push_back(lower_pt);
        out_pcl.points_.push_back(upper_pt);
        out_pcl.indexes_.emplace_back(i, j);
        out_pcl.indexes_.emplace_back(i + 1, j);
      }
    }
  }
}

void VelodyneMapper::groundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl)
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
        //        ground_mat_(i, j) = -1;
        continue;
      }

      float dX = upper_pt.x_ - lower_pt.x_;
      float dY = upper_pt.y_ - lower_pt.y_;
      float dZ = upper_pt.z_ - lower_pt.z_;

      float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

      if (vertical_angle <= ground_th_)
      {
        out_pcl.points_.push_back(lower_pt);
        out_pcl.points_.push_back(upper_pt);
        out_pcl.indexes_.emplace_back(i, j);
        out_pcl.indexes_.emplace_back(i + 1, j);
      }
    }
  }
}

void VelodyneMapper::cloudSegmentation(const std::vector<Point>& in_pts, std::vector<PlanePoint>& cloud_seg)
{
  // Segmentation process
  int label = 1;
  for (int i = 0; i < vertical_scans_; i++)
  {
    for (int j = 0; j < horizontal_scans_; j++)
    {
      if (label_mat_(i, j) == 0 && range_mat_(i, j) != -1)
        labelComponents(i, j, label);
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
          //          if (j % 20 != 0 && j > 20 && j < horizontal_scans_ - 20)
          if (j % 5 != 0 && j > 5 && j < horizontal_scans_ - 5)
          {
            continue;
          }
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

void VelodyneMapper::labelComponents(const int& row, const int& col, int& label)
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
  else if (static_cast<int>(global_queue.size()) >= segment_valid_point_num_)
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

void VelodyneMapper::extractHighLevelPlanes(const std::vector<Point>& in_pts, const SemiPlane& ground_plane,
                                            std::vector<SemiPlane>& out_planes)
{
  Tf tf;
  std::array<float, 9> tf_rot{};
  prev_robot_pose_.toRotMatrix(tf_rot);
  tf = Tf(tf_rot, std::array<float, 3>{ 0, 0, 0 });

  // Remove ground and null points from the set of input points
  std::vector<Point> non_ground{};
  for (const auto& pt : in_pts)
  {
    if (ground_plane.point2Plane(pt) > 0.2 && pt != Point(0, 0, 0))
    {
      non_ground.push_back(pt);
    }
  }

  // -------------------------------------------------------------------------------
  // ----- Segment plane points in two different sets
  // -------------------------------------------------------------------------------
  // - Start by computing the average of the y component of all points
  float y_mean = 0.;
  for (auto const& plane_pt : non_ground)
    y_mean += plane_pt.y_;
  y_mean /= static_cast<float>(non_ground.size());

  // - Cluster points using the y mean threshold
  Plane side_plane_a, side_plane_b;
  for (auto const& plane_pt : non_ground)
  {
    Point rotated_to_map_pt = plane_pt * tf;
    PlanePoint m_plane_pt;

    m_plane_pt.pos_ = plane_pt;
    m_plane_pt.which_plane_ = (rotated_to_map_pt.y_ < y_mean) ? 0 : 1;

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
  std::vector<Plane> planes = {};
  Plane side_plane_a_filtered, side_plane_b_filtered;
  if (Ransac::process(side_plane_a.points_, side_plane_a_filtered, 300, 0.10, true) &&
      side_plane_a_filtered.points_.size() < 7000 &&
      side_plane_a_filtered.points_.size() > 75)  // prevent dense planes and slow convex hulls
  {
    side_plane_a_filtered.id_ = 0;
    planes.push_back(side_plane_a_filtered);
  }
  if (Ransac::process(side_plane_b.points_, side_plane_b_filtered, 300, 0.10, true) &&
      side_plane_b_filtered.points_.size() < 7000 &&
      side_plane_b_filtered.points_.size() > 75)  // prevent dense planes and slow convex hulls
  {
    side_plane_b_filtered.id_ = 1;
    planes.push_back(side_plane_b_filtered);
  }

  // -------------------------------------------------------------------------------
  // ----- Check the validity of the extracted planes
  // -------------------------------------------------------------------------------
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
    bool ch = ConvexHull::process(plane, l_semi_plane);
    if (ch && checkPlaneConsistency(l_semi_plane, ground_plane))
    {
      out_planes.push_back(l_semi_plane);
    }
  }
}

bool VelodyneMapper::checkPlaneConsistency(const SemiPlane& plane, const SemiPlane& ground_plane)
{
  // A - Check semiplane area
  if (plane.area_ < 4)
  {
    return false;
  }

  // B - Check if the points belonging to the semiplane are continuous
  //  Point p0;
  //  if (plane.points_.empty())
  //  {
  //    return false;
  //  }
  //  else
  //  {
  //    p0 = plane.points_[0];
  //  }
  //  std::vector<Point> pts = plane.points_;
  //  float d0 = 0;
  //  while (pts.size() >= 2)  // Find nearest neighbor of p0, pop it from the vector, compare distances computed
  //                           // between iterations, find holes by large variations on the distance measured
  //  {
  //    float d1, min_dist = std::numeric_limits<float>::max();
  //    uint32_t idx = 0;
  //    for (uint32_t i = 0; i < pts.size(); ++i)
  //    {
  //      d1 = p0.distance(pts[i]);
  //      if (d1 != 0 && d1 < min_dist)
  //      {
  //        min_dist = d1;
  //        idx = i;
  //      }
  //    }
  //    d1 = min_dist;
  //    if (std::fabs(d1 - d0) > 0.2 && d0 != 0)  // We found a hole in this case ...
  //    {
  //      return false;
  //    }
  //    else
  //    {
  //      pts.erase(pts.begin() + idx);
  //      d0 = d1;
  //    }
  //  }

  // C - Make sure that the plane is not horizontal
  float dot = Vec(plane.a_, plane.b_, plane.c_).dot(Vec(ground_plane.a_, ground_plane.b_, ground_plane.c_));
  if (std::fabs(dot) > 1.0)
  {
    return false;
  }

  return true;
}

void VelodyneMapper::extractFeatures(const std::vector<PlanePoint>& in_plane_pts, std::vector<Corner>& out_corners,
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
            if (idx + m >= static_cast<int>(seg_pcl_.col_idx.size()))
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
void VelodyneMapper::reset()
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

void VelodyneMapper::localMap(const std::vector<Point>& pcl, std::vector<Corner>& out_corners,
                              std::vector<Planar>& out_planars, std::vector<SemiPlane>& out_planes,
                              SemiPlane& out_groundplane)
{
  // Build velodyne to base_link transformation matrix
  Pose tf_pose(laser2base_x_, laser2base_y_, laser2base_z_, laser2base_roll_, laser2base_pitch_, laser2base_yaw_);
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
    ang_res_y_ = static_cast<float>(2.) * DEGREE_TO_RAD;
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

    if (range > 50.0 || (std::fabs(l_pt.x_) < 0.9 && std::fabs(l_pt.y_) < 0.4))
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
  flatGroundRemoval(transformed_pcl, unfiltered_gplane);
  // B - Filtering
  Ransac::process(unfiltered_gplane.points_, filtered_gplane, 100, 0.01, true);
  // C - Centroid calculation
  for (const auto& pt : filtered_gplane.points_)
  {
    filtered_gplane.centroid_ = filtered_gplane.centroid_ + pt;
  }
  filtered_gplane.centroid_ = filtered_gplane.centroid_ / static_cast<float>(filtered_gplane.points_.size());
  // D - Bounding polygon
  ConvexHull::process(filtered_gplane, out_groundplane);
  for (auto& pt : out_groundplane.points_)
  {
    pt = pt * tf.inverse();
  }
  for (auto& pt : out_groundplane.extremas_)
  {
    pt = pt * tf.inverse();
  }
  out_groundplane.centroid_ = out_groundplane.centroid_ * tf.inverse();
  // E - Check plane consistency (set to null if not consistent)
  if (out_groundplane.area_ < 4.)
  {
    out_groundplane = SemiPlane();
  }

  // -------------------------------------------------------------------------------
  // ----- Mark raw ground points
  // -------------------------------------------------------------------------------
  Plane non_flat_ground;
  groundRemoval(transformed_pcl, non_flat_ground);
  for (const auto& index : non_flat_ground.indexes_)
  {
    int i = static_cast<int>(index.x_);
    int j = static_cast<int>(index.y_);

    ground_mat_(i, j) = 1;
    label_mat_(i, j) = -1;
  }

  // - Planes that are not the ground
  std::vector<PlanePoint> cloud_seg;
  cloudSegmentation(transformed_pcl, cloud_seg);

  // - Extract high level planes, and then convert them to semi-planes
  extractHighLevelPlanes(transformed_pcl, out_groundplane, out_planes);

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
}

// -------------------------------------------------------------------
// ----- Livox functions
// -------------------------------------------------------------------

LivoxMapper::LivoxMapper(const Parameters& params)
{
  // Set Livox configuration parameters
  pcl_data_save_index_ = 0;
  max_fov_ = 17;  // Edge of circle to main axis
  max_edge_polar_pos_ = 0;
  time_internal_pts_ = 1.0e-5;  // 10us = 1e-5
  cx_ = 0;
  cy_ = 0;
  if_save_pcd_file_ = 0;
  first_receive_time_ = -1;
  thr_corner_curvature_ = 0.1;
  thr_surface_curvature_ = 0.005;
  minimum_view_angle_ = 5;
  livox_min_allow_dis_ = 0.1;
  livox_min_sigma_ = 7e-4;

  // Set ground plane settings
  ground_th_ = static_cast<float>(3.) * DEGREE_TO_RAD;
  horizontal_scans_ = 1800;
  ground_scan_idx_ = 7;
  lidar_height_ = 1.20;
  vertical_scans_ = 16;
  horizontal_scans_ = 1800;
  vertical_angle_bottom_ = static_cast<float>(15. + 0.1) * DEGREE_TO_RAD;
  ang_res_x_ = static_cast<float>(0.2) * DEGREE_TO_RAD;
  ang_res_y_ = static_cast<float>(2.) * DEGREE_TO_RAD;

  // Set robot dimensions for elevation map computation
  robot_dim_x_ = params.robot_dim_x_;
  robot_dim_y_ = params.robot_dim_y_;
  robot_dim_z_ = params.robot_dim_z_;

  // Set previous robot pose handler
  prev_robot_pose_ = Pose(0, 0, 0, 0, 0, 0);
}

void LivoxMapper::localMap(const std::vector<Point>& pcl, const double& time_stamp, std::vector<Corner>& out_corners,
                           std::vector<Planar>& out_planars, std::vector<SemiPlane>& out_planes,
                           SemiPlane& out_groundplane)
{
  // Build velodyne to base_link transformation matrix
  Pose tf_pose(laser2base_x_, laser2base_y_, laser2base_z_, laser2base_roll_, laser2base_pitch_, laser2base_yaw_);
  Tf tf;
  std::array<float, 9> tf_rot{};
  tf_pose.toRotMatrix(tf_rot);
  tf = Tf(tf_rot, std::array<float, 3>{ tf_pose.x_, tf_pose.y_, tf_pose.z_ });

  // Extract livox features
  std::vector<std::vector<Point>> laser_cloud_scans = extractLaserFeatures(pcl, time_stamp);
  std::vector<Point> tmp_corners, tmp_planars, tmp_full;
  getFeatures(tmp_corners, tmp_planars, tmp_full);

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

    if (range > 50.0 || (std::fabs(l_pt.x_) < 0.9 && std::fabs(l_pt.y_) < 0.4))
    {
      continue;
    }

    size_t idx = column_idx + row_idx * horizontal_scans_;
    transformed_pcl[idx] = l_pt;
  }

  // Extract ground plane
  Plane unfiltered_gplane, filtered_gplane;
  flatGroundRemoval(transformed_pcl, unfiltered_gplane);
  // B - Filtering
  Ransac::process(unfiltered_gplane.points_, filtered_gplane, 100, 0.01, true);
  // C - Centroid calculation
  for (const auto& pt : filtered_gplane.points_)
  {
    filtered_gplane.centroid_ = filtered_gplane.centroid_ + pt;
  }
  filtered_gplane.centroid_ = filtered_gplane.centroid_ / static_cast<float>(filtered_gplane.points_.size());
  // D - Bounding polygon
  ConvexHull::process(filtered_gplane, out_groundplane);

  // - Extract high level planes, and then convert them to semi-planes
  extractHighLevelPlanes(transformed_pcl, out_groundplane, out_planes);

  // Convert features to VineSLAM type and project them to the base_link
  for (const auto& pt : tmp_corners)
  {
    Corner c(Point(pt.x_, pt.y_, pt.z_), 0);
    c.pos_ = c.pos_ * tf.inverse();
    out_corners.push_back(c);
  }
  for (const auto& pt : tmp_planars)
  {
    Planar p(Point(pt.x_, pt.y_, pt.z_), 0);
    p.pos_ = p.pos_ * tf.inverse();
    out_planars.push_back(p);
  }
  for (auto& pt : out_groundplane.points_)
  {
    pt = pt * tf.inverse();
  }
  for (auto& pt : out_groundplane.extremas_)
  {
    pt = pt * tf.inverse();
  }
  out_groundplane.centroid_ = out_groundplane.centroid_ * tf.inverse();
  // E - Check plane consistency (set to null if not consistent)
  if (out_groundplane.area_ < 4.)
  {
    out_groundplane = SemiPlane();
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
}

Pt_infos* LivoxMapper::findPtInfo(const Point& pt)
{
  map_pt_idx_it_ = map_pt_idx_.find(pt);
  if (map_pt_idx_it_ == map_pt_idx_.end())
  {
    assert(map_pt_idx_it_ != map_pt_idx_.end());  // else, there must be something error happened before.
  }
  return map_pt_idx_it_->second;
}

void LivoxMapper::getFeatures(std::vector<Point>& pc_corners, std::vector<Point>& pc_surface,
                              std::vector<Point>& pc_full_res, float minimum_blur, float maximum_blur)
{
  int corner_num = 0;
  int surface_num = 0;
  int full_num = 0;
  pc_corners.resize(pts_info_vec_.size());
  pc_surface.resize(pts_info_vec_.size());
  pc_full_res.resize(pts_info_vec_.size());
  float maximum_idx = maximum_blur * pts_info_vec_.size();
  float minimum_idx = minimum_blur * pts_info_vec_.size();
  int pt_critical_rm_mask = e_pt_000 | e_pt_nan | e_pt_too_near;
  for (size_t i = 0; i < pts_info_vec_.size(); i++)
  {
    if (pts_info_vec_[i].idx_ > maximum_idx || pts_info_vec_[i].idx_ < minimum_idx)
      continue;

    if ((pts_info_vec_[i].pt_type_ & pt_critical_rm_mask) == 0)
    {
      if (pts_info_vec_[i].pt_label_ & e_label_corner)
      {
        //        if (pts_info_vec_[i].pt_type_ != e_pt_normal)
        //          continue;
        if (pts_info_vec_[i].depth_sq2_ < std::pow(30, 2))
        {
          pc_corners[corner_num] = raw_pts_vec_[i];
          // set_intensity( pc_corners[ corner_num ], e_I_motion_blur );
          pc_corners[corner_num].intensity_ = pts_info_vec_[i].time_stamp_;
          corner_num++;
        }
      }
      if (pts_info_vec_[i].pt_label_ & e_label_surface)
      {
        if (pts_info_vec_[i].depth_sq2_ < std::pow(1000, 2))
        {
          pc_surface[surface_num] = raw_pts_vec_[i];
          pc_surface[surface_num].intensity_ = float(pts_info_vec_[i].time_stamp_);
          // set_intensity( pc_surface[ surface_num ], e_I_motion_blur );
          surface_num++;
        }
      }
    }
    pc_full_res[full_num] = raw_pts_vec_[i];
    pc_full_res[full_num].intensity_ = pts_info_vec_[i].time_stamp_;
    full_num++;
  }

  // printf("Get_features , corner num = %d, suface num = %d, blur from %.2f~%.2f\r\n", corner_num, surface_num,
  // minimum_blur, maximum_blur);
  pc_corners.resize(corner_num);
  pc_surface.resize(surface_num);
  pc_full_res.resize(full_num);
}

void LivoxMapper::setIntensity(Point& pt, const E_intensity_type& i_type)
{
  Pt_infos* pt_info = findPtInfo(pt);
  switch (i_type)
  {
    case (e_I_raw):
      pt.intensity_ = pt_info->raw_intensity_;
      break;
    case (e_I_motion_blur):
      pt.intensity_ = ((float)pt_info->idx_) / (float)input_points_size_;
      assert(pt.intensity_ <= 1.0 && pt.intensity_ >= 0.0);
      break;
    case (e_I_motion_mix):
      pt.intensity_ = 0.1 * ((float)pt_info->idx_ + 1) / (float)input_points_size_ + (int)(pt_info->raw_intensity_);
      break;
    case (e_I_scan_angle):
      pt.intensity_ = pt_info->polar_angle_;
      break;
    case (e_I_curvature):
      pt.intensity_ = pt_info->curvature_;
      break;
    case (e_I_view_angle):
      pt.intensity_ = pt_info->view_angle_;
      break;
    case (e_I_time_stamp):
      pt.intensity_ = pt_info->time_stamp_;
      break;
    default:
      pt.intensity_ = ((float)pt_info->idx_ + 1) / (float)input_points_size_;
      break;
  }
  return;
}

void LivoxMapper::addMaskOfPoint(Pt_infos* pt_infos, const E_point_type& pt_type_, int neighbor_count)
{
  int idx = pt_infos->idx_;
  pt_infos->pt_type_ |= pt_type_;

  if (neighbor_count > 0)
  {
    for (int i = -neighbor_count; i < neighbor_count; i++)
    {
      idx = pt_infos->idx_ + i;

      if (i != 0 && (idx >= 0) && (idx < (int)pts_info_vec_.size()))
      {
        pts_info_vec_[idx].pt_type_ |= pt_type_;
      }
    }
  }
}

void LivoxMapper::evalPoint(Pt_infos* pt_info)
{
  if (pt_info->depth_sq2_ < livox_min_allow_dis_ * livox_min_allow_dis_)  // to close
  {
    addMaskOfPoint(pt_info, e_pt_too_near);
  }

  pt_info->sigma_ = pt_info->raw_intensity_ / pt_info->polar_dis_sq2_;

  if (pt_info->sigma_ < livox_min_sigma_)
  {
    addMaskOfPoint(pt_info, e_pt_reflectivity_low);
  }
}

template <typename T>
T LivoxMapper::vector_angle(const Eigen::Matrix<T, 3, 1>& vec_a, const Eigen::Matrix<T, 3, 1>& vec_b,
                            int if_force_sharp_angle)
{
  T vec_a_norm = vec_a.norm();
  T vec_b_norm = vec_b.norm();
  if (vec_a_norm == 0 || vec_b_norm == 0)  // zero vector is pararrel to any vector.
  {
    return 0.0;
  }
  else
  {
    if (if_force_sharp_angle)
    {
      // return acos( abs( vec_a.dot( vec_b ) )*0.9999 / ( vec_a_norm * vec_b_norm ) );
      return acos(abs(vec_a.dot(vec_b)) / (vec_a_norm * vec_b_norm));
    }
    else
    {
      // return acos( (vec_a.dot(vec_b))*0.9999 / (vec_a_norm*vec_b_norm));
      return acos((vec_a.dot(vec_b)) / (vec_a_norm * vec_b_norm));
    }
  }
}

void LivoxMapper::computeFeatures()
{
  unsigned int pts_size = raw_pts_vec_.size();
  size_t curvature_ssd_size = 2;
  int critical_rm_point = e_pt_000 | e_pt_nan;
  float neighbor_accumulate_xyz[3] = { 0.0, 0.0, 0.0 };

  for (size_t idx = curvature_ssd_size; idx < pts_size - curvature_ssd_size; idx++)
  {
    if (pts_info_vec_[idx].pt_type_ & critical_rm_point)
    {
      continue;
    }

    // Compute curvature
    neighbor_accumulate_xyz[0] = 0.0;
    neighbor_accumulate_xyz[1] = 0.0;
    neighbor_accumulate_xyz[2] = 0.0;

    for (size_t i = 1; i <= curvature_ssd_size; i++)
    {
      if ((pts_info_vec_[idx + i].pt_type_ & e_pt_000) || (pts_info_vec_[idx - i].pt_type_ & e_pt_000))
      {
        if (i == 1)
        {
          pts_info_vec_[idx].pt_label_ |= e_label_near_zero;
        }
        else
        {
          pts_info_vec_[idx].pt_label_ = e_label_invalid;
        }
        break;
      }
      else if ((pts_info_vec_[idx + i].pt_type_ & e_pt_nan) || (pts_info_vec_[idx - i].pt_type_ & e_pt_nan))
      {
        if (i == 1)
        {
          pts_info_vec_[idx].pt_label_ |= e_label_near_nan;
        }
        else
        {
          pts_info_vec_[idx].pt_label_ = e_label_invalid;
        }
        break;
      }
      else
      {
        neighbor_accumulate_xyz[0] += raw_pts_vec_[idx + i].x_ + raw_pts_vec_[idx - i].x_;
        neighbor_accumulate_xyz[1] += raw_pts_vec_[idx + i].y_ + raw_pts_vec_[idx - i].y_;
        neighbor_accumulate_xyz[2] += raw_pts_vec_[idx + i].z_ + raw_pts_vec_[idx - i].z_;
      }
    }

    if (pts_info_vec_[idx].pt_label_ == e_label_invalid)
    {
      continue;
    }

    neighbor_accumulate_xyz[0] -= curvature_ssd_size * 2 * raw_pts_vec_[idx].x_;
    neighbor_accumulate_xyz[1] -= curvature_ssd_size * 2 * raw_pts_vec_[idx].y_;
    neighbor_accumulate_xyz[2] -= curvature_ssd_size * 2 * raw_pts_vec_[idx].z_;
    pts_info_vec_[idx].curvature_ = neighbor_accumulate_xyz[0] * neighbor_accumulate_xyz[0] +
                                    neighbor_accumulate_xyz[1] * neighbor_accumulate_xyz[1] +
                                    neighbor_accumulate_xyz[2] * neighbor_accumulate_xyz[2];

    // Compute plane angle
    Eigen::Matrix<float, 3, 1> vec_a(raw_pts_vec_[idx].x_, raw_pts_vec_[idx].y_, raw_pts_vec_[idx].z_);
    Eigen::Matrix<float, 3, 1> vec_b(
        raw_pts_vec_[idx + curvature_ssd_size].x_ - raw_pts_vec_[idx - curvature_ssd_size].x_,
        raw_pts_vec_[idx + curvature_ssd_size].y_ - raw_pts_vec_[idx - curvature_ssd_size].y_,
        raw_pts_vec_[idx + curvature_ssd_size].z_ - raw_pts_vec_[idx - curvature_ssd_size].z_);
    pts_info_vec_[idx].view_angle_ = vector_angle(vec_a, vec_b, 1) * 57.3;

    if (pts_info_vec_[idx].view_angle_ > minimum_view_angle_)
    {
      if (pts_info_vec_[idx].curvature_ < thr_surface_curvature_)
      {
        pts_info_vec_[idx].pt_label_ |= e_label_surface;
      }

      float sq2_diff = 0.1;

      if (pts_info_vec_[idx].curvature_ > thr_corner_curvature_)
      {
        if (pts_info_vec_[idx].depth_sq2_ <= pts_info_vec_[idx - curvature_ssd_size].depth_sq2_ &&
            pts_info_vec_[idx].depth_sq2_ <= pts_info_vec_[idx + curvature_ssd_size].depth_sq2_)
        {
          if (abs(pts_info_vec_[idx].depth_sq2_ - pts_info_vec_[idx - curvature_ssd_size].depth_sq2_) <
                  sq2_diff * pts_info_vec_[idx].depth_sq2_ ||
              abs(pts_info_vec_[idx].depth_sq2_ - pts_info_vec_[idx + curvature_ssd_size].depth_sq2_) <
                  sq2_diff * pts_info_vec_[idx].depth_sq2_)
            pts_info_vec_[idx].pt_label_ |= e_label_corner;
        }
      }
    }
  }
}

int LivoxMapper::projectionScan3d2d(const std::vector<Point>& laser_cloud_in, std::vector<float>& scan_id_index)
{
  unsigned int pts_size = laser_cloud_in.size();
  pts_info_vec_.clear();
  pts_info_vec_.resize(pts_size);
  raw_pts_vec_.resize(pts_size);
  std::vector<int> edge_idx;
  std::vector<int> split_idx;
  scan_id_index.resize(pts_size);
  map_pt_idx_.clear();
  map_pt_idx_.reserve(pts_size);
  std::vector<int> zero_idx;

  input_points_size_ = 0;

  for (unsigned int idx = 0; idx < pts_size; idx++)
  {
    raw_pts_vec_[idx] = laser_cloud_in[idx];
    Pt_infos* pt_info = &pts_info_vec_[idx];
    map_pt_idx_.insert(std::make_pair(laser_cloud_in[idx], pt_info));
    pt_info->raw_intensity_ = laser_cloud_in[idx].intensity_;
    pt_info->idx_ = idx;
    pt_info->time_stamp_ = current_time_ + ((float)idx) * time_internal_pts_;
    last_maximum_time_stamp_ = pt_info->time_stamp_;
    input_points_size_++;

    if (!std::isfinite(laser_cloud_in[idx].x_) || !std::isfinite(laser_cloud_in[idx].y_) ||
        !std::isfinite(laser_cloud_in[idx].z_))
    {
      addMaskOfPoint(pt_info, e_pt_nan);
      continue;
    }

    if (laser_cloud_in[idx].x_ == 0)
    {
      if (idx == 0)
      {
        // TODO: handle this case.

        pt_info->pt_2d_img_ << 0.01, 0.01;
        pt_info->polar_dis_sq2_ = 0.0001;
        addMaskOfPoint(pt_info, e_pt_000);
        // return 0;
      }
      else
      {
        pt_info->pt_2d_img_ = pts_info_vec_[idx - 1].pt_2d_img_;
        pt_info->polar_dis_sq2_ = pts_info_vec_[idx - 1].polar_dis_sq2_;
        addMaskOfPoint(pt_info, e_pt_000);
        continue;
      }
    }

    map_pt_idx_.insert(std::make_pair(laser_cloud_in[idx], pt_info));

    pt_info->depth_sq2_ = laser_cloud_in[idx].norm3D();

    pt_info->pt_2d_img_ << laser_cloud_in[idx].y_ / laser_cloud_in[idx].x_,
        laser_cloud_in[idx].z_ / laser_cloud_in[idx].x_;
    pt_info->polar_dis_sq2_ =
        pt_info->pt_2d_img_(0) * pt_info->pt_2d_img_(0) + pt_info->pt_2d_img_(1) * pt_info->pt_2d_img_(1);

    evalPoint(pt_info);

    if (pt_info->polar_dis_sq2_ > max_edge_polar_pos_)
    {
      addMaskOfPoint(pt_info, e_pt_circle_edge, 2);
    }

    // Split scans
    if (idx >= 1)
    {
      float dis_incre = pt_info->polar_dis_sq2_ - pts_info_vec_[idx - 1].polar_dis_sq2_;

      if (dis_incre > 0)  // far away from zero
      {
        pt_info->polar_direction_ = 1;
      }

      if (dis_incre < 0)  // move toward zero
      {
        pt_info->polar_direction_ = -1;
      }

      if (pt_info->polar_direction_ == -1 && pts_info_vec_[idx - 1].polar_direction_ == 1)
      {
        if (edge_idx.size() == 0 || (idx - split_idx[split_idx.size() - 1]) > 50)
        {
          split_idx.push_back(idx);
          edge_idx.push_back(idx);
          continue;
        }
      }

      if (pt_info->polar_direction_ == 1 && pts_info_vec_[idx - 1].polar_direction_ == -1)
      {
        if (zero_idx.size() == 0 || (idx - split_idx[split_idx.size() - 1]) > 50)
        {
          split_idx.push_back(idx);

          zero_idx.push_back(idx);
          continue;
        }
      }
    }
  }
  split_idx.push_back(pts_size - 1);

  int val_index = 0;
  int pt_angle_index = 0;
  float scan_angle = 0;
  int internal_size = 0;

  if (split_idx.size() < 6)  // minimum 3 petal of scan.
    return 0;

  for (int idx = 0; idx < (int)pts_size; idx++)
  {
    if (val_index < (int)split_idx.size() - 2)
    {
      if (idx == 0 || idx > split_idx[val_index + 1])
      {
        if (idx > split_idx[val_index + 1])
        {
          val_index++;
        }

        internal_size = split_idx[val_index + 1] - split_idx[val_index];

        if (pts_info_vec_[split_idx[val_index + 1]].polar_dis_sq2_ > 10000)
        {
          pt_angle_index = split_idx[val_index + 1] - (int)(internal_size * 0.20);
          scan_angle =
              atan2(pts_info_vec_[pt_angle_index].pt_2d_img_(1), pts_info_vec_[pt_angle_index].pt_2d_img_(0)) * 57.3;
          scan_angle = scan_angle + 180.0;
        }
        else
        {
          pt_angle_index = split_idx[val_index + 1] - (int)(internal_size * 0.80);
          scan_angle =
              atan2(pts_info_vec_[pt_angle_index].pt_2d_img_(1), pts_info_vec_[pt_angle_index].pt_2d_img_(0)) * 57.3;
          scan_angle = scan_angle + 180.0;
        }
      }
    }
    pts_info_vec_[idx].polar_angle_ = scan_angle;
    scan_id_index[idx] = scan_angle;
  }

  return split_idx.size() - 1;
}

// Split whole point cloud into scans.
void LivoxMapper::splitLaserScan(const int clutter_size, const std::vector<Point>& laser_cloud_in,
                                 const std::vector<float>& scan_id_index,
                                 std::vector<std::vector<Point>>& laser_cloud_scans)
{
  std::vector<std::vector<int>> pts_mask;
  laser_cloud_scans.resize(clutter_size);
  pts_mask.resize(clutter_size);
  Point point;
  int scan_idx = 0;

  for (unsigned int i = 0; i < laser_cloud_in.size(); i++)
  {
    point = laser_cloud_in[i];

    if (i > 0 && ((scan_id_index[i]) != (scan_id_index[i - 1])))
    {
      scan_idx = scan_idx + 1;
      pts_mask[scan_idx].reserve(5000);
    }

    laser_cloud_scans[scan_idx].push_back(point);
    pts_mask[scan_idx].push_back(pts_info_vec_[i].pt_type_);
  }
  laser_cloud_scans.resize(scan_idx);

  int remove_point_pt_type_ = e_pt_000 | e_pt_too_near | e_pt_nan;
  int scan_avail_num = 0;
  std::vector<std::vector<Point>> res_laser_cloud_scan;
  for (unsigned int i = 0; i < laser_cloud_scans.size(); i++)
  {
    scan_avail_num = 0;
    std::vector<Point> laser_clour_per_scan;
    for (unsigned int idx = 0; idx < laser_cloud_scans[i].size(); idx++)
    {
      if ((pts_mask[i][idx] & remove_point_pt_type_) == 0)
      {
        if (laser_cloud_scans[i][idx].x_ == 0)
        {
          assert(laser_cloud_scans[i][idx].x_ != 0);
          continue;
        }
        auto temp_pt = laser_cloud_scans[i][idx];
        setIntensity(temp_pt, default_return_intensity_type_);
        laser_clour_per_scan.push_back(temp_pt);
        scan_avail_num++;
      }
    }

    if (scan_avail_num)
    {
      res_laser_cloud_scan.push_back(laser_clour_per_scan);
    }
  }
  laser_cloud_scans = res_laser_cloud_scan;
}

std::vector<std::vector<Point>> LivoxMapper::extractLaserFeatures(const std::vector<Point>& laser_cloud_in,
                                                                  double time_stamp)
{
  assert(time_stamp >= 0.0);
  if (time_stamp <= 0.0000001 || (time_stamp < last_maximum_time_stamp_))  // old firmware, without timestamp
  {
    current_time_ = last_maximum_time_stamp_;
  }
  else
  {
    current_time_ = time_stamp - first_receive_time_;
  }
  if (first_receive_time_ <= 0)
  {
    first_receive_time_ = time_stamp;
  }

  std::vector<std::vector<Point>> laser_cloud_scans, temp_laser_scans;
  std::vector<float> scan_id_index;
  laser_cloud_scans.clear();
  map_pt_idx_.clear();
  pts_info_vec_.clear();
  raw_pts_vec_.clear();

  //  if (if_save_pcd_file)
  //  {
  //    stringstream ss;
  //    ss << PCL_DATA_SAVE_DIR << "/pc_" << pcl_data_save_index_ << ".pcd" << endl;
  //    pcl_data_save_index_ = pcl_data_save_index_ + 1;
  //    screen_out << "Save file = " << ss.str() << endl;
  //    pcl::io::savePCDFileASCII(ss.str(), laser_cloud_in);
  //  }

  int clutter_size = projectionScan3d2d(laser_cloud_in, scan_id_index);
  computeFeatures();
  if (clutter_size == 0)
  {
    return laser_cloud_scans;
  }
  else
  {
    splitLaserScan(clutter_size, laser_cloud_in, scan_id_index, laser_cloud_scans);
    return laser_cloud_scans;
  }
}

void LivoxMapper::flatGroundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl)
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

      //      if (range_mat_(i, j) == -1 || range_mat_(i + 1, j) == -1)
      //      {
      //        // no info to check, invalid points
      //        //        ground_mat_(i, j) = -1;
      //        continue;
      //      }

      float dX = upper_pt.x_ - lower_pt.x_;
      float dY = upper_pt.y_ - lower_pt.y_;
      float dZ = upper_pt.z_ - lower_pt.z_;

      float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

      if (vertical_angle <= ground_th_ && std::fabs(lower_pt.z_) > lidar_height_ / 2 &&
          std::fabs(upper_pt.z_) > lidar_height_ / 2)
      {
        out_pcl.points_.push_back(lower_pt);
        out_pcl.points_.push_back(upper_pt);
        out_pcl.indexes_.emplace_back(i, j);
        out_pcl.indexes_.emplace_back(i + 1, j);
      }
    }
  }
}

void LivoxMapper::extractHighLevelPlanes(const std::vector<Point>& in_pts, const SemiPlane& ground_plane,
                                         std::vector<SemiPlane>& out_planes)
{
  Tf tf;
  std::array<float, 9> tf_rot{};
  prev_robot_pose_.toRotMatrix(tf_rot);
  tf = Tf(tf_rot, std::array<float, 3>{ 0, 0, 0 });

  // Remove ground and null points from the set of input points
  std::vector<Point> non_ground{};
  for (const auto& pt : in_pts)
  {
    if (ground_plane.point2Plane(pt) > 0.2 && pt != Point(0, 0, 0))
    {
      non_ground.push_back(pt);
    }
  }

  // -------------------------------------------------------------------------------
  // ----- Segment plane points in two different sets
  // -------------------------------------------------------------------------------
  // - Start by computing the average of the y component of all points
  float y_mean = 0.;
  for (auto const& plane_pt : non_ground)
    y_mean += plane_pt.y_;
  y_mean /= static_cast<float>(non_ground.size());

  // - Cluster points using the y mean threshold
  Plane side_plane_a, side_plane_b;
  for (auto const& plane_pt : non_ground)
  {
    Point rotated_to_map_pt = plane_pt * tf;
    PlanePoint m_plane_pt;

    m_plane_pt.pos_ = plane_pt;
    m_plane_pt.which_plane_ = (rotated_to_map_pt.y_ < y_mean) ? 0 : 1;

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
  std::vector<Plane> planes = {};
  Plane side_plane_a_filtered, side_plane_b_filtered;
  if (Ransac::process(side_plane_a.points_, side_plane_a_filtered, 300, 0.10, true) &&
      side_plane_a_filtered.points_.size() < 7000 &&
      side_plane_a_filtered.points_.size() > 75)  // prevent dense planes and slow convex hulls
  {
    side_plane_a_filtered.id_ = 0;
    planes.push_back(side_plane_a_filtered);
  }
  if (Ransac::process(side_plane_b.points_, side_plane_b_filtered, 300, 0.10, true) &&
      side_plane_b_filtered.points_.size() < 7000 &&
      side_plane_b_filtered.points_.size() > 75)  // prevent dense planes and slow convex hulls
  {
    side_plane_b_filtered.id_ = 1;
    planes.push_back(side_plane_b_filtered);
  }

  // -------------------------------------------------------------------------------
  // ----- Check the validity of the extracted planes
  // -------------------------------------------------------------------------------
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
    bool ch = ConvexHull::process(plane, l_semi_plane);
    if (ch && checkPlaneConsistency(l_semi_plane, ground_plane))
    {
      out_planes.push_back(l_semi_plane);
    }
  }
}

bool LivoxMapper::checkPlaneConsistency(const SemiPlane& plane, const SemiPlane& ground_plane)
{
  // A - Check semiplane area
  if (plane.area_ < 4)
  {
    return false;
  }

  // B - Check if the points belonging to the semiplane are continuous
  //  Point p0;
  //  if (plane.points_.empty())
  //  {
  //    return false;
  //  }
  //  else
  //  {
  //    p0 = plane.points_[0];
  //  }
  //  std::vector<Point> pts = plane.points_;
  //  float d0 = 0;
  //  while (pts.size() >= 2)  // Find nearest neighbor of p0, pop it from the vector, compare distances computed
  //                           // between iterations, find holes by large variations on the distance measured
  //  {
  //    float d1, min_dist = std::numeric_limits<float>::max();
  //    uint32_t idx = 0;
  //    for (uint32_t i = 0; i < pts.size(); ++i)
  //    {
  //      d1 = p0.distance(pts[i]);
  //      if (d1 != 0 && d1 < min_dist)
  //      {
  //        min_dist = d1;
  //        idx = i;
  //      }
  //    }
  //    d1 = min_dist;
  //    if (std::fabs(d1 - d0) > 0.2 && d0 != 0)  // We found a hole in this case ...
  //    {
  //      return false;
  //    }
  //    else
  //    {
  //      pts.erase(pts.begin() + idx);
  //      d0 = d1;
  //    }
  //  }

  // C - Make sure that the plane is not horizontal
  float dot = Vec(plane.a_, plane.b_, plane.c_).dot(Vec(ground_plane.a_, ground_plane.b_, ground_plane.c_));
  if (std::fabs(dot) > 1.0)
  {
    return false;
  }

  return true;
}

}  // namespace vineslam