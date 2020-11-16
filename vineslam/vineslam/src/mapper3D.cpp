#include "mapper3D.hpp"
#include <chrono>
#include <set>

namespace vineslam
{

Mapper3D::Mapper3D(const Parameters& params)
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

  // Threshold to consider correspondences
  correspondence_threshold = 0.02;

  // Set velodyne configuration parameters
  picked_num              = 2;
  planes_th               = static_cast<float>(45.) * DEGREE_TO_RAD;
  ground_th               = static_cast<float>(5.) * DEGREE_TO_RAD;
  edge_threshold          = 0.1;
  planar_threshold        = 0.1;
  vertical_scans          = 16;
  horizontal_scans        = 1800;
  ground_scan_idx         = 7;
  segment_valid_point_num = 5;
  segment_valid_line_num  = 3;
  vertical_angle_bottom   = static_cast<float>(15. + 0.1) * DEGREE_TO_RAD;
  ang_res_x               = static_cast<float>(0.2) * DEGREE_TO_RAD;
  ang_res_y               = static_cast<float>(2.) * DEGREE_TO_RAD;
  lidar_height            = 1.20;
}

void Mapper3D::registerMaps(const pose&                robot_pose,
                            std::vector<ImageFeature>& img_features,
                            std::vector<Corner>&       corners,
                            std::vector<Planar>&       planars,
                            std::vector<Plane>&        planes,
                            OccupancyMap&              grid_map)
{
  // - 3D image feature map estimation
  globalSurfMap(img_features, robot_pose, grid_map);
  // - 3D PCL corner map estimation
  globalCornerMap(robot_pose, corners, grid_map);
  // - 3D PCL corner map estimation
  globalPlanarMap(robot_pose, planars, grid_map);
}

// -------------------------------------------------------------------------------
// ---- 3D image feature map functions
// -------------------------------------------------------------------------------

void Mapper3D::localSurfMap(const cv::Mat&             img,
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

void Mapper3D::globalSurfMap(const std::vector<ImageFeature>& features,
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
    float        best_correspondence = correspondence_threshold;
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

void Mapper3D::extractSurfFeatures(const cv::Mat&             in,
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

void Mapper3D::pixel2base(const point& in_pt,
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

// -------------------------------------------------------------------------------
// ---- 3D pointcloud feature map functions
// -------------------------------------------------------------------------------

void Mapper3D::reset()
{
  range_mat.resize(vertical_scans, horizontal_scans);
  ground_mat.resize(vertical_scans, horizontal_scans);
  label_mat.resize(vertical_scans, horizontal_scans);
  range_mat.fill(-1);
  ground_mat.setZero();
  label_mat.setZero();

  int cloud_size = vertical_scans * horizontal_scans;

  seg_pcl.start_col_idx.assign(vertical_scans, 0);
  seg_pcl.end_col_idx.assign(vertical_scans, 0);
  seg_pcl.is_ground.assign(cloud_size, false);
  seg_pcl.col_idx.assign(cloud_size, 0);
  seg_pcl.range.assign(cloud_size, 0);
}

void Mapper3D::localPCLMap(const std::vector<point>& pcl,
                           std::vector<Corner>&      out_corners,
                           std::vector<Planar>&      out_planars,
                           std::vector<Plane>&       out_planes,
                           Plane&                    out_groundplane)
{
  std::vector<point> transformed_pcl;

  // Reset global variables and members
  reset();

  // Range image projection
  const size_t cloud_size = pcl.size();
  transformed_pcl.resize(vertical_scans * horizontal_scans);
  for (size_t i = 0; i < cloud_size; ++i) {
    point m_pt = pcl[i];

    float range = m_pt.norm3D();

    // find the row and column index in the image for this point
    float vertical_angle =
        std::atan2(m_pt.z, std::sqrt(m_pt.x * m_pt.x + m_pt.y * m_pt.y));

    int row_idx =
        static_cast<int>((vertical_angle + vertical_angle_bottom) / ang_res_y);
    if (row_idx < 0 || row_idx >= vertical_scans) {
      continue;
    }

    float horizon_angle = std::atan2(m_pt.x, m_pt.y); // this is not an error

    int column_idx = static_cast<int>(-round((horizon_angle - M_PI_2) / ang_res_x) +
                                      horizontal_scans / 2.);

    if (column_idx >= horizontal_scans) {
      column_idx -= horizontal_scans;
    }

    if (column_idx < 0 || column_idx >= horizontal_scans) {
      continue;
    }

    if (range < 1.0 && range > 25.0) {
      continue;
    }

    range_mat(row_idx, column_idx) = range;

    size_t idx           = column_idx + row_idx * horizontal_scans;
    transformed_pcl[idx] = m_pt;
  }

  // - GROUND PLANE
  groundRemoval(transformed_pcl, out_groundplane);

  // -------------------------------------------------------------------------------
  // ----- Mark ground points
  // -------------------------------------------------------------------------------
  for (const auto& index : out_groundplane.indexes) {
    int i = static_cast<int>(index.x);
    int j = static_cast<int>(index.y);

    ground_mat(i, j) = 1;
    label_mat(i, j)  = -1;
  }

  // - Planes that are not the ground
  std::vector<PlanePoint> cloud_seg, cloud_seg_pure;
  cloudSegmentation(transformed_pcl, cloud_seg, cloud_seg_pure);

  // - Extract two planes (right and left) from point cloud
  extractHighLevelPlanes(cloud_seg, out_planes);

  //- Corners feature extraction
  extract3DFeatures(cloud_seg, out_corners, out_planars);

  // - Convert local maps to base link
  pose tf_pose;
  TF   tf;

  std::array<float, 9> tf_rot{};
  tf_pose.toRotMatrix(tf_rot);
  tf = TF(tf_rot, std::array<float, 3>{vel2base_x, vel2base_y, vel2base_z});

  for (auto& pt : out_groundplane.points) {
    pt = pt * tf.inverse();
  }
  for (auto& corner : out_corners) {
    corner.pos = corner.pos * tf.inverse();
  }
  for (auto& planar : out_planars) {
    planar.pos = planar.pos * tf.inverse();
  }
  for (auto& plane : out_planes) {
    for (auto& pt : plane.points) {
      pt = pt * tf.inverse();
    }
  }
}

void Mapper3D::globalCornerMap(const pose&          robot_pose,
                               std::vector<Corner>& corners,
                               OccupancyMap&        grid_map) const
{
  // ------ Convert robot pose into homogeneous transformation
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = {robot_pose.x, robot_pose.y, robot_pose.z};
  TF                   tf(Rot, trans);

  // ------ Insert corner into the grid map
  for (auto& corner : corners) {
    // - First convert them to map's referential using the robot pose
    point m_pt = corner.pos * tf;

    // - Then, look for correspondences in the local map
    Corner              correspondence{};
    float               best_correspondence = correspondence_threshold;
    bool                found               = false;
    std::vector<Corner> m_corners = grid_map(m_pt.x, m_pt.y, m_pt.z).corner_features;
    for (const auto& m_corner : m_corners) {
      float dist_min = m_pt.distance(m_corner.pos);

      if (dist_min < best_correspondence) {
        correspondence      = m_corner;
        best_correspondence = dist_min;
        found               = true;
      }
    }

    found &= (best_correspondence < 0.02);

    // - Then, insert the corner into the grid map
    if (found) {
      point new_pt =
          ((correspondence.pos * static_cast<float>(correspondence.n_observations)) +
           m_pt) /
          static_cast<float>(correspondence.n_observations + 1);
      Corner new_corner(new_pt, corner.which_plane);
      new_corner.n_observations = correspondence.n_observations + 1;
      grid_map.update(correspondence, new_corner);
    } else {
      Corner new_corner(m_pt, corner.which_plane);
      grid_map.insert(new_corner);
    }
  }
}

void Mapper3D::globalPlanarMap(const pose&          robot_pose,
                               std::vector<Planar>& planars,
                               OccupancyMap&        grid_map) const
{
  // ------ Convert robot pose into homogeneous transformation
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = {robot_pose.x, robot_pose.y, robot_pose.z};
  TF                   tf(Rot, trans);

  // ------ Insert planar into the grid map
  for (auto& planar : planars) {
    // - First convert them to map's referential using the robot pose
    point m_pt = planar.pos * tf;

    // - Then, look for correspondences in the local map
    Planar              correspondence{};
    float               best_correspondence = correspondence_threshold;
    bool                found               = false;
    std::vector<Planar> m_planars = grid_map(m_pt.x, m_pt.y, m_pt.z).planar_features;
    for (const auto& m_planar : m_planars) {
      float dist_min = m_pt.distance(m_planar.pos);

      if (dist_min < best_correspondence) {
        correspondence      = m_planar;
        best_correspondence = dist_min;
        found               = true;
      }
    }

    found &= (best_correspondence < 0.02);

    // - Then, insert the planar into the grid map
    if (found) {
      point new_pt =
          ((correspondence.pos * static_cast<float>(correspondence.n_observations)) +
           m_pt) /
          static_cast<float>(correspondence.n_observations + 1);
      Planar new_planar(new_pt, planar.which_plane);
      new_planar.n_observations = correspondence.n_observations + 1;
      grid_map.update(correspondence, new_planar);
    } else {
      Planar new_planar(m_pt, planar.which_plane);
      grid_map.insert(new_planar);
    }
  }
}

void Mapper3D::groundRemoval(const std::vector<point>& in_pts, Plane& out_pcl)
{
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (int j = 0; j < horizontal_scans; j++) {
    for (int i = 0; i < ground_scan_idx; i++) {
      int lower_idx = j + i * horizontal_scans;
      int upper_idx = j + (i + 1) * horizontal_scans;

      point upper_pt = in_pts[upper_idx];
      point lower_pt = in_pts[lower_idx];

      if (range_mat(i, j) == -1 || range_mat(i + 1, j) == -1) {
        // no info to check, invalid points
        ground_mat(i, j) = -1;
        continue;
      }

      float dX = upper_pt.x - lower_pt.x;
      float dY = upper_pt.y - lower_pt.y;
      float dZ = upper_pt.z - lower_pt.z;

      float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

      if (vertical_angle <= ground_th && std::fabs(lower_pt.z) > lidar_height / 2 &&
          std::fabs(upper_pt.z) > lidar_height / 2) {
        out_pcl.points.push_back(lower_pt);
        out_pcl.points.push_back(upper_pt);
        out_pcl.indexes.emplace_back(i, j);
        out_pcl.indexes.emplace_back(i + 1, j);
      }
    }
  }
}

void Mapper3D::cloudSegmentation(const std::vector<point>& in_pts,
                                 std::vector<PlanePoint>&  cloud_seg,
                                 std::vector<PlanePoint>&  cloud_seg_pure)
{
  // Segmentation process
  int label = 1;
  for (int i = 0; i < vertical_scans; i++) {
    for (int j = 0; j < horizontal_scans; j++) {
      if (label_mat(i, j) == 0 && range_mat(i, j) != -1)
        labelComponents(i, j, in_pts, label);
    }
  }

  // Extract segmented cloud for visualization
  int seg_cloud_size = 0;
  for (int i = 0; i < vertical_scans; i++) {

    seg_pcl.start_col_idx[i] = seg_cloud_size - 1 + 5;

    for (int j = 0; j < horizontal_scans; j++) {
      if (label_mat(i, j) > 0 || ground_mat(i, j) == 1) {
        if (label_mat(i, j) == 999999)
          continue;

        // The majority of ground points are skipped
        if (ground_mat(i, j) == 1) {
//          if (j % 5 != 0 && j > 5 && j < horizontal_scans - 5) {
            continue;
//          }
        }

        // Mark ground points so they will not be considered as edge features later
        seg_pcl.is_ground[seg_cloud_size] = (ground_mat(i, j) == 1);

        // Save segmented cloud into a pcl
        point      pt = in_pts[j + i * horizontal_scans];
        PlanePoint m_ppoint(pt, label_mat(i, j));
        cloud_seg.push_back(m_ppoint);
        // ------------------------------------------
        // Save segmented cloud in the given structure
        seg_pcl.col_idx[seg_cloud_size] = j;
        seg_pcl.range[seg_cloud_size]   = range_mat(i, j);
        seg_cloud_size++;
        // ------------------------------------------
      }
    }
    seg_pcl.end_col_idx[i] = seg_cloud_size - 1 - 5;
  }
}

void Mapper3D::labelComponents(const int&                row,
                               const int&                col,
                               const std::vector<point>& in_pts,
                               int&                      label)
{
  using Coord2D = Eigen::Vector2i;
  std::deque<Coord2D> queue;
  std::deque<Coord2D> global_queue;

  queue.emplace_back(row, col);
  global_queue.emplace_back(row, col);

  std::vector<bool> line_count_flag(vertical_scans, false);

  // - Define neighborhood
  const Coord2D neighbor_it[4] = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  while (!queue.empty()) {
    // Evaluate front element of the queue and pop it
    Coord2D from_idx = queue.front();
    queue.pop_front();

    // Mark popped point as belonging to the segment
    label_mat(from_idx.x(), from_idx.y()) = label;

    // Compute point from range image
    float d1 = range_mat(from_idx.x(), from_idx.y());

    // Loop through all the neighboring grids of popped grid
    for (const auto& iter : neighbor_it) {
      // Compute new index
      int c_idx_x = from_idx.x() + iter.x();
      int c_idx_y = from_idx.y() + iter.y();

      // Check if index is within the boundary
      if (c_idx_x < 0 || c_idx_x >= vertical_scans)
        continue;
      if (c_idx_y < 0)
        c_idx_y = horizontal_scans - 1;
      if (c_idx_y >= horizontal_scans)
        c_idx_y = 0;

      // Prevent infinite loop (caused by put already examined point back)
      if (label_mat(c_idx_x, c_idx_y) != 0)
        continue;

      // Compute point from range image
      float d2 = range_mat(c_idx_x, c_idx_y);

      float dmax = std::max(d1, d2);
      float dmin = std::min(d1, d2);

      // Compute angle between the two points
      float alpha = (iter.x() == 0) ? ang_res_x : ang_res_y;

      // Compute beta and check if points belong to the same segment
      auto beta =
          std::atan2((dmin * std::sin(alpha)), (dmax - dmin * std::cos(alpha)));
      if (beta > planes_th) {
        queue.emplace_back(c_idx_x, c_idx_y);
        global_queue.emplace_back(c_idx_x, c_idx_y);

        label_mat(c_idx_x, c_idx_y) = label;
        line_count_flag[c_idx_x]    = true;
      }
    }
  }

  // Check if this segment is valid
  bool feasible_segment = false;
  if (global_queue.size() >= 30) {
    feasible_segment = true;
  } else if (global_queue.size() >= segment_valid_point_num) {
    int line_count = 0;
    for (int i = 0; i < vertical_scans; i++) {
      if (line_count_flag[i])
        line_count++;
    }

    if (line_count >= segment_valid_line_num)
      feasible_segment = true;
  }

  if (feasible_segment) {
    label++;
  } else {
    for (auto& i : global_queue) label_mat(i.x(), i.y()) = 999999;
  }
}

void Mapper3D::extractHighLevelPlanes(const std::vector<PlanePoint>& in_plane_pts,
                                      std::vector<Plane>&            out_planes)
{
  // -------------------------------------------------------------------------------
  // ----- Segment plane points in two different sets
  // -------------------------------------------------------------------------------
  // - Start by computing the average of the y component of all points
  float y_mean = 0.;
  for (auto const& plane_pt : in_plane_pts) y_mean += plane_pt.pos.y;
  y_mean /= static_cast<float>(in_plane_pts.size());

  // - Cluster points using the y mean threshold
  Plane side_plane_a, side_plane_b;
  for (auto const& plane_pt : in_plane_pts) {
    PlanePoint m_plane_pt = plane_pt;

    m_plane_pt.which_plane = (m_plane_pt.pos.y < y_mean) ? 0 : 1;

    if (m_plane_pt.which_plane == 0) {
      side_plane_a.points.push_back(m_plane_pt.pos);
    } else {
      side_plane_b.points.push_back(m_plane_pt.pos);
    }
  }

  // - Remove outliers using RANSAC
  Plane side_plane_a_filtered, side_plane_b_filtered;
  side_plane_a_filtered.ransac(side_plane_a, 100);
  side_plane_b_filtered.ransac(side_plane_b, 100);
  side_plane_a_filtered.id = 0;
  side_plane_b_filtered.id = 1;

  // -------------------------------------------------------------------------------
  // ----- Check the validity of the extracted planes
  // -------------------------------------------------------------------------------
  std::vector<Plane> planes = {side_plane_a_filtered, side_plane_b_filtered};

  for (auto plane : planes) {
    bool A = false, B = false;

    // (A) - Check if the plane have a minimum number of points
    if (plane.points.size() > 500)
      A = true;

    // (B) - Compute point to line distance and check if the average distance is
    //       bellow a threshold
    float dist = 0;
    if (!plane.points.empty()) {
      for (const auto& pt : plane.points) {
        dist += plane.regression.dist(pt);
      }
      dist /= static_cast<float>(plane.points.size());
    }

    if (dist > 0 && dist < 0.05)
      B = true;

    // (C) - Save plane if in case of success
    if (A && B)
      out_planes.push_back(plane);
  }
}

void Mapper3D::extract3DFeatures(const std::vector<PlanePoint>& in_plane_pts,
                                 std::vector<Corner>&           out_corners,
                                 std::vector<Planar>&           out_planars)

{
  // -------------------------------------------------------------------------------
  // ----- Compute cloud smoothness
  // -------------------------------------------------------------------------------
  int* cloudPlanarLabel = new int[vertical_scans * horizontal_scans];
  int* cloudCornerLabel = new int[vertical_scans * horizontal_scans];
  int  m_cloud_size     = in_plane_pts.size();
  std::vector<smoothness_t> cloud_smoothness(vertical_scans * horizontal_scans);
  std::vector<int>          neighbor_picked(vertical_scans * horizontal_scans);
  for (int i = 5; i < m_cloud_size - 5; i++) {
    // Compute smoothness and save it
    float diff_range =
        seg_pcl.range[i - 5] + seg_pcl.range[i - 4] + seg_pcl.range[i - 3] +
        seg_pcl.range[i - 2] + seg_pcl.range[i - 1] + seg_pcl.range[i + 1] +
        seg_pcl.range[i + 2] + seg_pcl.range[i + 3] + seg_pcl.range[i + 4] +
        seg_pcl.range[i + 5] - 10 * seg_pcl.range[i];

    cloud_smoothness[i].value = diff_range * diff_range;
    cloud_smoothness[i].idx   = i;

    cloudPlanarLabel[i] = 0;
    cloudCornerLabel[i] = 0;

    // Reset neighborhood flag array
    neighbor_picked[i] = 0;
  }

  // -------------------------------------------------------------------------------
  // ----- Mark occluded points
  // -------------------------------------------------------------------------------

  for (int i = 5; i < m_cloud_size - 6; ++i) {

    float depth1   = seg_pcl.range[i];
    float depth2   = seg_pcl.range[i + 1];
    int   col_diff = std::abs(int(seg_pcl.col_idx[i + 1] - seg_pcl.col_idx[i]));

    if (col_diff < 10) {

      if (depth1 - depth2 > 0.3) {
        neighbor_picked[i - 5] = 1;
        neighbor_picked[i - 4] = 1;
        neighbor_picked[i - 3] = 1;
        neighbor_picked[i - 2] = 1;
        neighbor_picked[i - 1] = 1;
        neighbor_picked[i]     = 1;
      } else if (depth2 - depth1 > 0.3) {
        neighbor_picked[i + 1] = 1;
        neighbor_picked[i + 2] = 1;
        neighbor_picked[i + 3] = 1;
        neighbor_picked[i + 4] = 1;
        neighbor_picked[i + 5] = 1;
        neighbor_picked[i + 6] = 1;
      }
    }

    float diff1 = std::abs(float(seg_pcl.range[i - 1] - seg_pcl.range[i]));
    float diff2 = std::abs(float(seg_pcl.range[i + 1] - seg_pcl.range[i]));

    if (diff1 > 0.02 * seg_pcl.range[i] && diff2 > 0.02 * seg_pcl.range[i])
      neighbor_picked[i] = 1;
  }

  // -------------------------------------------------------------------------------
  // ----- Extract features from the 3D cloud
  // -------------------------------------------------------------------------------
  std::vector<Planar> planar_points_less_flat;
  int                 corner_id = 0;
  int                 planar_id = 0;
  for (int i = 0; i < vertical_scans; i++) {

    planar_points_less_flat.clear();

    for (int k = 0; k < 6; k++) {
      // Compute start and end indexes of the sub-region
      int sp =
          (seg_pcl.start_col_idx[i] * (6 - k) + (seg_pcl.end_col_idx[i] * k)) / 6;
      int ep =
          (seg_pcl.start_col_idx[i] * (5 - k) + (seg_pcl.end_col_idx[i] * (k + 1))) /
              6 -
          1;

      if (sp >= ep)
        continue;

      // Sort smoothness values for the current sub-region
      std::sort(
          cloud_smoothness.begin() + sp, cloud_smoothness.begin() + ep, by_value());

      // -- Extract edge features
      int picked_counter = 0;
      for (int l = ep; l >= sp; l--) {
        int idx = cloud_smoothness[l].idx;

        // Check if the current point is an edge feature
        if (neighbor_picked[idx] == 0 &&
            cloud_smoothness[l].value > edge_threshold && !seg_pcl.is_ground[idx]) {
          picked_counter++;
          if (picked_counter <= picked_num) {
            Corner m_corner(
                in_plane_pts[idx].pos, in_plane_pts[idx].which_plane, corner_id);
            out_corners.push_back(m_corner);
            corner_id++;
          } else {
            break;
          }

          cloudCornerLabel[idx] = -1;

          // Mark neighbor points to reject as future features
          neighbor_picked[idx] = 1;
          for (int m = 1; m <= 5; m++) {
            if (idx + m >= seg_pcl.col_idx.size())
              continue;
            int col_diff =
                std::abs(seg_pcl.col_idx[idx + m] - seg_pcl.col_idx[idx + m - 1]);
            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
          for (int m = -1; m >= -5; m--) {
            if (idx + m < 0)
              continue;
            int col_diff =
                std::abs(seg_pcl.col_idx[idx + m] - seg_pcl.col_idx[idx + m + 1]);
            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
        }
      }

      // -- Extract planar features
      picked_counter = 0;
      for (int l = sp; l <= ep; l++) {
        int idx = cloud_smoothness[l].idx;

        // Check if the current point is a planar feature
        if (neighbor_picked[idx] == 0 &&
            cloud_smoothness[l].value < planar_threshold && seg_pcl.is_ground[idx]) {

          cloudPlanarLabel[idx] = -1;

          picked_counter++;
          if (picked_counter >= 4)
            break;

          neighbor_picked[idx] = 1;
          for (int m = 1; m <= 5; m++) {
            int col_diff =
                std::abs(seg_pcl.col_idx[idx + m] - seg_pcl.col_idx[idx + m - 1]);

            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
          for (int m = -1; m >= -5; m--) {
            if (idx + m < 0)
              continue;
            int col_diff =
                std::abs(seg_pcl.col_idx[idx + m] - seg_pcl.col_idx[idx + m + 1]);
            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
        }
      }

      for (int l = sp; l <= ep; l++) {
        if (cloudPlanarLabel[l] <= 0 && cloudCornerLabel[l] >= 0) {
          Planar m_planar(
              in_plane_pts[l].pos, in_plane_pts[l].which_plane, planar_id);
          planar_points_less_flat.push_back(m_planar);
        }
      }
    }

    out_planars.insert(out_planars.end(),
                       planar_points_less_flat.begin(),
                       planar_points_less_flat.end());
  }
}

} // namespace vineslam
