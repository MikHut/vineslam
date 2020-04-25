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
    point3D point_map(x,
                      y,
                      depths[idx],
                      cam_height,
                      fx,
                      fy,
                      cx,
                      cy,
                      rot_matrix,
                      point3D(sensor_origin.x, sensor_origin.y, sensor_origin.z));

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
