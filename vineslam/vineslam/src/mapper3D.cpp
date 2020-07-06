#include "mapper3D.hpp"

namespace vineslam
{

Mapper3D::Mapper3D(const std::string& config_path)
{
  // Load configuration file
  YAML::Node config = YAML::LoadFile(config_path);

  // Load camera info parameters
  img_width  = config["camera_info"]["img_width"].as<int>();
  img_height = config["camera_info"]["img_height"].as<int>();
  cam_height = config["camera_info"]["cam_height"].as<float>();
  cam_pitch =
      config["camera_info"]["cam_pitch"].as<float>() * static_cast<float>(PI / 180.);
  fx              = config["camera_info"]["fx"].as<float>();
  fy              = config["camera_info"]["fy"].as<float>();
  cx              = config["camera_info"]["cx"].as<float>();
  cy              = config["camera_info"]["cy"].as<float>();
  auto depth_hfov = config["camera_info"]["depth_hfov"].as<float>() *
                    static_cast<float>(PI / 180.);
  auto depth_vfov = config["camera_info"]["depth_vfov"].as<float>() *
                    static_cast<float>(PI / 180.);
  // Load 3D map parameters
  correspondence_threshold =
      config["map_3D"]["correspondence_threshold"].as<float>();
  max_range  = config["map_3D"]["max_range"].as<float>();
  max_height = config["map_3D"]["max_height"].as<float>();
  // Feature detector
  fdetector = config["image_feature"]["type"].as<std::string>();
  // Load pointcloud feature parameters
  downsample_f = config["cloud_feature"]["downsample_factor"].as<int>();
  planes_th    = config["cloud_feature"]["planes_theta"].as<float>() *
              static_cast<float>(PI / 180.);
  ground_th = config["cloud_feature"]["ground_theta"].as<float>() *
              static_cast<float>(PI / 180.);
  max_iters      = config["cloud_feature"]["RANSAC"]["max_iters"].as<int>();
  dist_threshold = config["cloud_feature"]["RANSAC"]["dist_threshold"].as<float>();
  edge_threshold = config["cloud_feature"]["edge_threshold"].as<float>();

  // Compute x and y angle resolution of depth image
  angle_hres = depth_hfov / static_cast<float>(img_width);
  angle_vres = depth_vfov / static_cast<float>(img_height);
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
    pixel2world(in_pt, cam_pitch, m_depth, out_pt);
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
                             OccupancyMap&                    grid_map)
{
  // ------ Convert robot pose into homogeneous transformation
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = {robot_pose.x, robot_pose.y, robot_pose.z};

  // ------ Insert features into the grid map
  for (const auto& feature : features) {
    // - First convert them to map's referential using the robot pose
    ImageFeature m_feature = feature;

    point m_pt;
    m_pt.x = feature.pos.x * Rot[0] + feature.pos.y * Rot[1] +
             feature.pos.z * Rot[2] + trans[0];
    m_pt.y = feature.pos.x * Rot[3] + feature.pos.y * Rot[4] +
             feature.pos.z * Rot[5] + trans[1];
    m_pt.z = feature.pos.x * Rot[6] + feature.pos.y * Rot[7] +
             feature.pos.z * Rot[8] + trans[2];

    m_feature.pos = m_pt;

    // - Then, insert the feature into the grid map
    grid_map.insert(m_feature);
  }
}

void Mapper3D::extractSurfFeatures(const cv::Mat& in, std::vector<ImageFeature>& out)
{
  using namespace cv::xfeatures2d;

  // Array to store the features
  std::vector<cv::KeyPoint> kpts;
  // String to store the type of feature
  std::string type;
  // Matrix to store the descriptor
  cv::Mat desc;

  // Perform feature extraction
  auto surf = SURF::create(1800);
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

// -------------------------------------------------------------------------------
// ---- 3D pointcloud feature map functions
// -------------------------------------------------------------------------------

void Mapper3D::localPCLMap(const float*         depths,
                           std::vector<Corner>& out_corners,
                           Plane&               out_groundplane)
{
  // Reset global variables and members
  reset();

  std::vector<point> pts3D(img_width * img_height);
  for (auto i = 0; i < img_width; i++) {
    for (auto j = 0; j < img_height; j++) {
      int idx = i + img_width * j;

      float m_depth = depths[idx];

      // Check validity of depth information
      if (!std::isfinite(m_depth) || m_depth > max_range) {
        range_mat(i, j)          = -1;
        pts3D[i + j * img_width] = point(-1, -1, -1);
        continue;
      }

      // Pixel to 3D point conversion
      point out_pt;
      pixel2world(point(i, j), cam_pitch, m_depth, out_pt);
      // Save point and range
      pts3D[i + j * img_width] = out_pt;
      range_mat(i, j) =
          std::sqrt(out_pt.x * out_pt.x + out_pt.y * out_pt.y + out_pt.z * out_pt.z);
    }
  }

  // - GROUND PLANE
  Plane gplane_unfilt;
  groundRemoval(pts3D, gplane_unfilt);
  // Filter outliers using RANSAC
  ransac(gplane_unfilt, out_groundplane);

  // - OTHER PLANES
  std::vector<PlanePoint> cloud_seg;
  cloudSegmentation(pts3D, cloud_seg);

  //- Feature extraction and publication
  extractCorners(cloud_seg, out_corners);
}

void Mapper3D::globalCornerMap(const std::vector<Corner>& corners,
                               const pose&                robot_pose,
                               OccupancyMap&              grid_map) const
{
  // ------ Convert robot pose into homogeneous transformation
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = {robot_pose.x, robot_pose.y, robot_pose.z};

  // ------ Insert corner into the grid map
  for (const auto& corner : corners) {
    // - First convert them to map's referential using the robot pose
    point m_pt;
    m_pt.x = corner.pos.x * Rot[0] + corner.pos.y * Rot[1] + corner.pos.z * Rot[2] +
             trans[0];
    m_pt.y = corner.pos.x * Rot[3] + corner.pos.y * Rot[4] + corner.pos.z * Rot[5] +
             trans[1];
    m_pt.z = corner.pos.x * Rot[6] + corner.pos.y * Rot[7] + corner.pos.z * Rot[8] +
             trans[2];

    // - Then, look for correspondences in the local map
    Corner correspondence{};
    float  best_correspondence = correspondence_threshold;
    bool   found               = false;
    for (const auto& m_corner : grid_map(m_pt.x, m_pt.y).corner_features) {
      float dist_min = m_pt.distance(m_corner.pos);

      if (dist_min < best_correspondence) {
        correspondence      = m_corner;
        best_correspondence = dist_min;
        found               = true;
      }
    }

    // Only search in the adjacent cells if we do not find in the source cell
    if (!found) {
      std::vector<Cell> adjacents;
      grid_map.getAdjacent(m_pt.x, m_pt.y, 2, adjacents);
      for (const auto& m_cell : adjacents) {
        for (const auto& m_corner : m_cell.corner_features) {
          float dist_min = m_pt.distance(m_corner.pos);
          if (dist_min < best_correspondence) {
            correspondence      = m_corner;
            best_correspondence = dist_min;
            found               = true;
          }
        }
      }
    }

    // - Then, insert the corner into the grid map
    if (found) {
      point  new_pt = (m_pt + correspondence.pos) / 2.;
      Corner new_corner(new_pt, corner.which_plane);
      grid_map.update(correspondence, new_corner);
    } else {
      Corner new_corner(m_pt, corner.which_plane);
      grid_map.insert(new_corner);
    }
  }
}

void Mapper3D::reset()
{
  range_mat.resize(img_width, img_height);
  ground_mat.resize(img_width, img_height);
  label_mat.resize(img_width, img_height);
  range_mat.setZero();
  ground_mat.setZero();
  label_mat.setZero();

  int cloud_size = img_width * img_height;

  seg_pcl.start_col_idx.assign(img_height, 0);
  seg_pcl.end_col_idx.assign(img_height, 0);
  seg_pcl.is_ground.assign(cloud_size, false);
  seg_pcl.col_idx.assign(cloud_size, 0);
  seg_pcl.range.assign(cloud_size, 0);
}

void Mapper3D::groundRemoval(const std::vector<point>& in_pts, Plane& out_pcl)
{
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  int xlim = img_width;
  int ylim = img_height;
  int ymin = img_height / 2;
  for (int i = 0; i < xlim;) {
    for (int j = ymin; j < ylim - 1;) {
      int upper_idx = i + j * img_width;
      int lower_idx = i + (j + 1) * img_width;

      point upper_pt = in_pts[upper_idx];
      point lower_pt = in_pts[lower_idx];

      if (upper_pt.z == -1 || lower_pt.z == -1) {
        // no info to check, invalid points
        ground_mat(i, j) = -1;
        j += downsample_f;
        continue;
      }

      float dX = upper_pt.x - lower_pt.x;
      float dY = upper_pt.y - lower_pt.y;
      float dZ = upper_pt.z - lower_pt.z;

      float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

      if ((vertical_angle/* - cam_pitch*/) <= ground_th) {
        ground_mat(i, j)     = 1;
        ground_mat(i, j + 1) = 1;
        label_mat(i, j)      = -1;
        label_mat(i, j + 1)  = -1;

        point m_lower_pt(lower_pt.x, lower_pt.y, lower_pt.z);
        point m_upper_pt(upper_pt.x, upper_pt.y, upper_pt.z);

        out_pcl.points.push_back(m_lower_pt);
        out_pcl.points.push_back(m_upper_pt);
      }

      j += downsample_f;
    }
    i += downsample_f;
  }
}

bool Mapper3D::ransac(const Plane& in_plane, Plane& out_plane) const
{
  int   max_idx       = in_plane.points.size();
  int   min_idx       = 0;
  int   max_tries     = 1000;
  int   c_max_inliers = 0;
  float best_a        = 0.;
  float best_b        = 0.;
  float best_c        = 0.;
  float best_d        = 0.;

  for (int i = 0; i < max_iters; i++) {
    // Declare private point cloud to store current solution
    std::vector<point> m_pcl;

    // Reset number of inliers in each iteration
    int num_inliers = 0;

    // Randomly select three points that cannot be cohincident
    // TODO (André Aguiar): Also check if points are collinear
    bool found_valid_pts = false;
    int  n               = 0;
    int  idx1, idx2, idx3;
    while (!found_valid_pts) {
      idx1 = std::rand() % (max_idx - min_idx + 1) + min_idx;
      idx2 = std::rand() % (max_idx - min_idx + 1) + min_idx;
      idx3 = std::rand() % (max_idx - min_idx + 1) + min_idx;

      if (idx1 != idx2 && idx1 != idx3 && idx2 != idx3)
        found_valid_pts = true;

      n++;
      if (n > max_tries)
        break;
    }

    if (!found_valid_pts) {
      std::cout << "WARNING (ransac): No valid set of points found ... "
                << std::endl;
      return false;
    }

    // Declarate the 3 points selected on this iteration
    point pt1 = point(
        in_plane.points[idx1].x, in_plane.points[idx1].y, in_plane.points[idx1].z);
    point pt2 = point(
        in_plane.points[idx2].x, in_plane.points[idx2].y, in_plane.points[idx2].z);
    point pt3 = point(
        in_plane.points[idx3].x, in_plane.points[idx3].y, in_plane.points[idx3].z);

    // Extract the plane hessian coefficients
    vector3D v1(pt2, pt1);
    vector3D v2(pt3, pt1);
    vector3D abc = v1.cross(v2);
    float    a   = abc.x;
    float    b   = abc.y;
    float    c   = abc.z;
    float    d   = -(a * pt1.x + b * pt1.y + c * pt1.z);

    for (const auto& m_pt : in_plane.points) {
      // Compute the distance each point to the plane - from
      // https://www.geeksforgeeks.org/distance-between-a-point-and-a-plane-in-3-d/
      auto norm = std::sqrt(a * a + b * b + c * c);
      if (std::fabs(a * m_pt.x + b * m_pt.y + c * m_pt.z + d) / norm <
          dist_threshold) {
        num_inliers++;
        m_pcl.push_back(m_pt);
      }
    }

    if (num_inliers > c_max_inliers) {
      c_max_inliers = num_inliers;

      best_a = a;
      best_b = b;
      best_c = c;
      best_d = d;

      out_plane.points.clear();
      out_plane.points = m_pcl;
    }
  }

  // -------------------------------------------------------------------------------
  // ----- Use PCA to refine th normal vector using all the inliers
  // -------------------------------------------------------------------------------
  // - 1st: assemble data matrix
  Eigen::MatrixXf data_mat;
  data_mat.conservativeResize(out_plane.points.size(), 3);
  for (size_t i = 0; i < out_plane.points.size(); i++) {
    point                      pt = out_plane.points[i];
    Eigen::Matrix<float, 1, 3> pt_mat(pt.x, pt.y, pt.z);
    data_mat.block<1, 3>(i, 0) = pt_mat;
  }
  // - 2nd: calculate mean and subtract it to the data matrix
  Eigen::MatrixXf centered_mat = data_mat.rowwise() - data_mat.colwise().mean();
  // - 3rd: calculate covariance matrix
  Eigen::MatrixXf covariance_mat = (centered_mat.adjoint() * centered_mat);
  // - 4rd: calculate eigenvectors and eigenvalues of the covariance matrix
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigen_solver(covariance_mat);
  Eigen::VectorXf eigen_values  = eigen_solver.eigenvalues();
  Eigen::MatrixXf eigen_vectors = eigen_solver.eigenvectors();

  vector3D normal(
      eigen_vectors.col(0)[0], eigen_vectors.col(0)[1], eigen_vectors.col(0)[2]);
  if (normal.z < 0) {
    vector3D             m_vec = normal;
    std::array<float, 9> Rot{};
    pose                 transf(0., 0., 0., 0., PI, 0.);
    transf.toRotMatrix(Rot);

    m_vec.x = normal.x * Rot[0] + normal.y * Rot[1] + normal.z * Rot[2];
    m_vec.y = normal.x * Rot[3] + normal.y * Rot[4] + normal.z * Rot[5];
    m_vec.z = normal.x * Rot[6] + normal.y * Rot[7] + normal.z * Rot[8];

    normal = m_vec;
  }

  normal.normalize();
  out_plane.normal = normal;

  return c_max_inliers > 0;
}

void Mapper3D::cloudSegmentation(const std::vector<point>& in_pts,
                                 std::vector<PlanePoint>&  out_plane_pts)
{
  int xlim = img_width;
  int ylim = img_height;

  // Segmentation process
  int label = 1;
  for (int i = 0; i < xlim;) {
    for (int j = 0; j < ylim - 1;) {
      if (label_mat(i, j) == 0)
        labelComponents(i, j, in_pts, label);
      j += downsample_f;
    }
    i += downsample_f;
  }

  // Extract segmented cloud for visualization
  int seg_cloud_size = 0;
  for (int j = 0; j < ylim - 1; j++) {
    seg_pcl.start_col_idx[j] = seg_cloud_size - 1 + 5;
    for (int i = 0; i < xlim; i++) {
      if (label_mat(i, j) > 0 && label_mat(i, j) != 999999) {
        // Save segmented cloud into a pcl
        point      pt = in_pts[i + j * img_width];
        PlanePoint m_ppoint(pt, label_mat(i, j));
        out_plane_pts.push_back(m_ppoint);
        // ------------------------------------------
        // Save segmented cloud in the given structure
        seg_pcl.col_idx[seg_cloud_size] = j;
        seg_pcl.range[seg_cloud_size]   = range_mat(i, j);
        seg_cloud_size++;
        // ------------------------------------------
      }
    }
    seg_pcl.end_col_idx[j] = seg_cloud_size - 1 - 5;
  }
}

void Mapper3D::labelComponents(const int&                row,
                               const int&                col,
                               const std::vector<point>& in_pts,
                               int&                      label)
{
  float theta_threshold = std::tan(planes_th);

  using Coord2D = Eigen::Vector2i;
  std::deque<Coord2D> queue;
  std::deque<Coord2D> global_queue;

  queue.emplace_back(row, col);
  global_queue.emplace_back(row, col);

  // - Define neighborhood
  const Coord2D neighbor_it[4] = {
      {0, -downsample_f}, {-downsample_f, 0}, {downsample_f, 0}, {0, downsample_f}};

  while (!queue.empty()) {
    // Evaluate front element of the queue and pop it
    Coord2D from_idx = queue.front();
    queue.pop_front();

    // Mark popped point as belonging to the segment
    label_mat(from_idx.x(), from_idx.y()) = label;

    // Compute point from disparity
    point p1 = in_pts[from_idx.x() + img_width * from_idx.y()];
    if (p1.z == -1)
      continue;

    // Compute point one range
    float d1 = std::sqrt(p1.x * p1.x + p1.y * p1.y + p1.z * p1.z);

    // Loop through all the neighboring grids of popped grid
    for (const auto& iter : neighbor_it) {
      // Compute new index
      int c_idx_x = from_idx.x() + iter.x();
      int c_idx_y = from_idx.y() + iter.y();

      // Check if index is within the boundary
      if (c_idx_x < 0 || c_idx_y < 0 || c_idx_x >= img_width ||
          c_idx_y >= img_height)
        continue;

      // Prevent infinite loop (caused by put already examined point back)
      if (label_mat(c_idx_x, c_idx_y) != 0)
        continue;

      // Compute point from disparity
      point p2 = in_pts[c_idx_x + img_width * c_idx_y];
      if (p2.z == -1)
        continue;

      auto  d2   = std::sqrt(p2.x * p2.x + p2.y * p2.y + p2.z * p2.z);
      float dmax = std::max(d1, d2);
      float dmin = std::min(d1, d2);

      // Compute angle between the two points
      // NOTE: In a LiDAR/Laser case scenario, here we would use the angular
      // resolution of the sensor
      float alpha = (iter.y() == 0) ? angle_hres : angle_vres;

      // Compute beta and check if points belong to the same segment
      auto beta = (dmin * std::sin(alpha)) / (dmax - dmin * std::cos(alpha));
      if (beta > theta_threshold) {
        queue.emplace_back(c_idx_x, c_idx_y);
        global_queue.emplace_back(c_idx_x, c_idx_y);

        label_mat(c_idx_x, c_idx_y) = label;
      }
    }
  }

  if (global_queue.size() >= 30) {
    label++;
  } else {
    for (auto& i : global_queue) label_mat(i.x(), i.y()) = 999999;
  }
}

void Mapper3D::extractCorners(const std::vector<PlanePoint>& in_plane_pts,
                              std::vector<Corner>&           out_corners)
{
  // -------------------------------------------------------------------------------
  // ----- Compute cloud smoothness
  // -------------------------------------------------------------------------------
  int                       cloud_size = in_plane_pts.size();
  std::vector<smoothness_t> cloud_smoothness(img_width * img_height);
  std::vector<int>          neighbor_picked(img_width * img_height);
  for (int i = 5; i < cloud_size; i++) {
    // Compute smoothness and save it
    float diff_range =
        seg_pcl.range[i - 5] + seg_pcl.range[i - 4] + seg_pcl.range[i - 3] +
        seg_pcl.range[i - 2] + seg_pcl.range[i - 1] + seg_pcl.range[i + 1] +
        seg_pcl.range[i + 2] + seg_pcl.range[i + 3] + seg_pcl.range[i + 4] +
        seg_pcl.range[i + 5] - 10 * seg_pcl.range[i];

    cloud_smoothness[i].value = diff_range * diff_range;
    cloud_smoothness[i].idx   = i;

    // Reset neighborhood flag array
    neighbor_picked[i] = 0;
  }

  // -------------------------------------------------------------------------------
  // ----- Extract features from the 3D cloud
  // -------------------------------------------------------------------------------
  for (int j = 0; j < img_height; j++) {
    for (int k = 0; k < 6; k++) {
      // Compute start and end indexes of the sub-region
      int sp =
          (seg_pcl.start_col_idx[j] * (6 - k) + (seg_pcl.end_col_idx[j] * k)) / 6;
      int ep =
          (seg_pcl.start_col_idx[j] * (5 - k) + (seg_pcl.end_col_idx[j] * (k + 1))) /
              6 -
          1;

      if (sp >= ep)
        continue;

      // Sort smoothness values for the current sub-region
      std::sort(
          cloud_smoothness.begin() + sp, cloud_smoothness.begin() + ep, by_value());

      // -- Extract edge features
      // -----------------------------------------------------
      int picked_counter = 0;
      for (int l = ep; l >= sp; l--) {
        int idx = cloud_smoothness[l].idx;

        // Check if the current point is an edge feature
        if (neighbor_picked[idx] == 0 &&
            cloud_smoothness[l].value > edge_threshold) {
          picked_counter++;
          if (picked_counter > 2)
            break;
          else {
            Corner m_corner(in_plane_pts[idx].pos, in_plane_pts[idx].which_plane);
            out_corners.push_back(m_corner);
            neighbor_picked[idx] = 1;
          }

          // Mark neighbor points to reject as future features
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
      // -----------------------------------------------------
    }
  }
}

// -------------------------------------------------------------------------------

void Mapper3D::pixel2world(const point& in_pt,
                           const float& pitch,
                           const float& depth,
                           point&       out_pt) const
{
  // Project 2D pixel into a 3D Point using the stereo depth information
  float x_cam = (in_pt.x - cx) * (depth / fx);
  float y_cam = (in_pt.y - cy) * (depth / fy);
  float z_cam = depth;

  // Compute camera-world axis transformation matrix
  // - NOTE: We compensate here the camera height and pitch (!)
  pose                 transform(0., 0., cam_height, -PI / 2. - pitch, 0., -PI / 2.);
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

} // namespace vineslam
