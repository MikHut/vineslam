#include "mapper_3d.hpp"

using namespace octomap;

Mapper3D::Mapper3D(const std::string& config_path)
{
  // Load configuration file
  YAML::Node config = YAML::LoadFile(config_path.c_str());

  // Load camera info parameters
  img_width  = config["camera_info"]["img_width"].as<float>();
  img_height = config["camera_info"]["img_height"].as<float>();
  cam_height = config["camera_info"]["cam_height"].as<float>();
  fx         = config["camera_info"]["fx"].as<float>();
  fy         = config["camera_info"]["fy"].as<float>();
  cx         = config["camera_info"]["cx"].as<float>();
  cy         = config["camera_info"]["cy"].as<float>();
  // Load octree parameters
  max_range = config["mapper3D"]["max_range"].as<float>();
}

void Mapper3D::trunkMap(OcTreeT*                             octree,
                        const float*                         depths,
                        cv::Mat                              image,
                        pose6D                               sensor_origin,
                        const vision_msgs::Detection2DArray& dets)
{
  // Define the minimum and maximum levels of disparity to consider
  float range_min = 0.01;
  float range_max = 10.0;

  // Declare 3D point and intensities vectors
  std::vector<point3D>                pts_array;
  std::vector<std::array<uint8_t, 3>> ints_array;

  // Compute camera-world axis align matrix
  pose6D             align_pose(0., 0., cam_height, -PI / 2, 0., -PI / 2);
  std::vector<float> c2w_rot;
  align_pose.toRotMatrix(c2w_rot);

  // Convert pose6D to rotation matrix
  std::vector<float> rot_matrix;
  sensor_origin.toRotMatrix(rot_matrix);

  // Loop over all the bounding boxes
  for (size_t n = 0; n < dets.detections.size(); n++) {
    // Load a single bounding box detection
    vision_msgs::BoundingBox2D m_bbox = dets.detections[n].bbox;
    // Compute the limites of the bounding boxes
    float xmin = m_bbox.center.x - m_bbox.size_x / 2;
    float xmax = m_bbox.center.x + m_bbox.size_x / 2;
    float ymin = m_bbox.center.y - m_bbox.size_y / 2;
    float ymax = m_bbox.center.y + m_bbox.size_y / 2;

    // Initialize and compute average depth inside the bbox
    float mean_depth = 0;
    int   it         = 0;
    for (int x = xmin; x < xmax; x++) {
      for (int y = ymin; y < ymax; y++) {
        int idx = x + img_width * y;
        // Check if the current disparity value is valid
        if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
            depths[idx] < range_max) {
          mean_depth += depths[idx];
          it++;
        }
      }
    }
    mean_depth /= (float)it;

    for (int x = xmin; x < xmax; x++) {
      for (int y = ymin; y < ymax; y++) {
        int idx = x + img_width * y;
        // Check if the current disparity value is valid
        if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
            depths[idx] < mean_depth) {
          // Compute the 3D point considering the disparity and
          // the camera parameters
          point3D point;
          point.z = depths[idx];
          point.x = (float)(x - cx) * (point.z / fx);
          point.y = (float)(y - cy) * (point.z / fy);

          // Camera to map point cloud conversion
          // -------------------------------------------------------
          // Align world and camera axis
          point3D point_cam;
          point_cam.x = c2w_rot[0] * point.x + c2w_rot[1] * point.y +
                        c2w_rot[2] * point.z + align_pose.x;
          point_cam.y = c2w_rot[3] * point.x + c2w_rot[4] * point.y +
                        c2w_rot[5] * point.z + align_pose.y;
          point_cam.z = c2w_rot[6] * point.x + c2w_rot[7] * point.y +
                        c2w_rot[8] * point.z + align_pose.z;
          // -------------------------------------------------------
          // Apply robot pose to convert points to map's referential
          // frame
          point3D point_map;
          point_map.x = point_cam.x * rot_matrix[0] + point_cam.y * rot_matrix[1] +
                        point_cam.z * rot_matrix[2] + sensor_origin.x;
          point_map.y = point_cam.x * rot_matrix[3] + point_cam.y * rot_matrix[4] +
                        point_cam.z * rot_matrix[5] + sensor_origin.y;
          point_map.z = point_cam.x * rot_matrix[6] + point_cam.y * rot_matrix[7] +
                        point_cam.z * rot_matrix[8] + sensor_origin.z;
          // -------------------------------------------------------

          // Fill the point cloud array
          pts_array.push_back(point_map);

          // Access to the pixel intensity corresponding to the 2D pixel
          // that was converted to a 3D point
          cv::Point3_<uchar>* p = image.ptr<cv::Point3_<uchar>>(y, x);
          // Store the RGB value
          std::array<uint8_t, 3> m_rgb;
          m_rgb[0] = (*p).z;
          m_rgb[1] = (*p).y;
          m_rgb[2] = (*p).x;
          // Save the RGB value into the multi array
          ints_array.push_back(m_rgb);
        }
      }
    }
  }

  // Create octomap using the built PCL and return the corresponding
  // octree
  updateOctoMap(octree, sensor_origin, pts_array, ints_array);
}

void Mapper3D::featureMap(OcTreeT*                    octree,
                          const float*                depths,
                          cv::Mat                     image,
                          pose6D                      sensor_origin,
                          const std::vector<Feature>& features)
{
  // Declare inputs for map creation
  std::vector<point3D>                pts;
  std::vector<std::array<uint8_t, 3>> ints;

  // Compute camera-world axis align matrix
  pose6D             align_pose(0., 0., cam_height, -PI / 2, 0., -PI / 2);
  std::vector<float> c2w_rot;
  align_pose.toRotMatrix(c2w_rot);

  // Convert pose6D to rotation matrix
  std::vector<float> rot_matrix;
  sensor_origin.toRotMatrix(rot_matrix);

  for (size_t i = 0; i < features.size(); i++) {
    // Compute depth of image feature
    int   x     = features[i].u;
    int   y     = features[i].v;
    int   idx   = x + image.cols * y;
    float depth = depths[idx];

    // Check validity of depth information
    if (!std::isfinite(depths[idx]))
      continue;

    // Project 2D feature into 3D world point in
    // camera's referential frame
    point3D point;
    point.x = (float)((x - cx) * (depth / fx));
    point.y = (float)((y - cy) * (depth / fy));
    point.z = depth;

    // Camera to map point cloud conversion
    // -------------------------------------------------------
    // Align world and camera axis
    point3D point_cam;
    point_cam.x = c2w_rot[0] * point.x + c2w_rot[1] * point.y +
                  c2w_rot[2] * point.z + align_pose.x;
    point_cam.y = c2w_rot[3] * point.x + c2w_rot[4] * point.y +
                  c2w_rot[5] * point.z + align_pose.y;
    point_cam.z = c2w_rot[6] * point.x + c2w_rot[7] * point.y +
                  c2w_rot[8] * point.z + align_pose.z;
    // -------------------------------------------------------
    // Apply robot pose to convert points to map's referential
    // frame
    point3D point_map;
    point_map.x = point_cam.x * rot_matrix[0] + point_cam.y * rot_matrix[1] +
                  point_cam.z * rot_matrix[2] + sensor_origin.x;
    point_map.y = point_cam.x * rot_matrix[3] + point_cam.y * rot_matrix[4] +
                  point_cam.z * rot_matrix[5] + sensor_origin.y;
    point_map.z = point_cam.x * rot_matrix[6] + point_cam.y * rot_matrix[7] +
                  point_cam.z * rot_matrix[8] + sensor_origin.z;
    // -------------------------------------------------------

    // Store information on arrays
    //----------------------------------------------------------------
    point3D m_pt;
    m_pt.x = point_map.x;
    m_pt.y = point_map.y;
    m_pt.z = point_map.z;
    pts.push_back(m_pt);
    //----------------------------------------------------------------
    cv::Point3_<uchar>*    p = image.ptr<cv::Point3_<uchar>>(y, x);
    std::array<uint8_t, 3> m_rgb;
    m_rgb[0] = (*p).z;
    m_rgb[1] = (*p).y;
    m_rgb[2] = (*p).x;
    ints.push_back(m_rgb);
    //----------------------------------------------------------------
  }

  // Build 3D map
  updateOctoMap(octree, sensor_origin, pts, ints);
}

void Mapper3D::updateOctoMap(OcTreeT*                    octree,
                             pose6D&                     sensor_origin,
                             const std::vector<point3D>& pcl,
                             const std::vector<std::array<uint8_t, 3>>& ints)
{
  // Declare cells structures to fill
  KeySet occupied_cells;

  // Convert sensor origin to octomap format
  octomap::point3d m_sensor_origin(
      sensor_origin.x, sensor_origin.y, sensor_origin.z);

  // Loop over all the point in the point cloud
  for (size_t i = 0; i < pcl.size(); i++) {
    // Load the current point
    point3d point(pcl[i].x, pcl[i].y, pcl[i].z);

    // Check max range
    if ((max_range < 0.0) || ((point - m_sensor_origin).norm() <= max_range)) {

      OcTreeKey key;
      if ((*octree).coordToKeyChecked(point, key)) {
        // Insert the cell into the KeySet
        occupied_cells.insert(key);

        // Declare the 3-channel RGB color for the voxel
        uint8_t r = ints[i][0];
        uint8_t g = ints[i][1];
        uint8_t b = ints[i][2];

        // Give color to the voxel
        (*octree).averageNodeColor(key, r, g, b);
      }
    }
  }

  // Mark all occupied cells
  for (KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end();
       it != end;
       it++) {
    (*octree).updateNode(*it, true);
  }
}
