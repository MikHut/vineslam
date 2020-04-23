#include "localizer.hpp"

Localizer::Localizer(const std::string& config_path)
    : config_path(config_path)
{
  // Read input parameters
  YAML::Node config = YAML::LoadFile(config_path.c_str());
  cam_pitch         = config["camera_info"]["cam_pitch"].as<double>() * PI / 180;
  img_width         = config["camera_info"]["img_width"].as<float>();
  img_height        = config["camera_info"]["img_height"].as<float>();
  cam_height        = config["camera_info"]["cam_height"].as<float>();
  fx                = config["camera_info"]["fx"].as<float>();
  fy                = config["camera_info"]["fy"].as<float>();
  cx                = config["camera_info"]["cx"].as<float>();
  cy                = config["camera_info"]["cy"].as<float>();
  n_particles       = config["pf"]["n_particles"].as<int>();
}

void Localizer::init(const pose6D& initial_pose)
{
  // Initialize the particle filter
  pf = new PF(config_path, n_particles, initial_pose);

  // Get the first distribution of particles
  std::vector<Particle> particles;
  (*pf).getParticles(particles);

  // Compute average pose and standard deviation of the
  // first distribution
  std::vector<pose6D> poses;
  for (size_t i = 0; i < particles.size(); i++) poses.push_back(particles[i].pose);
  average_pose = pose6D(poses);
}

void Localizer::process(const pose6D&                         odom,
                        const std::vector<float>&             bearings2D,
                        const std::vector<float>&             landmark_depths,
                        const std::map<int, Landmark<float>>& map,
                        float*                                feature_depths,
                        const std::vector<Feature>&           features)
{
  // Invocate the particle filter loop
  (*pf).process(odom, bearings2D, landmark_depths, map, feature_depths, features);
  // Import the resultant set of particles
  std::vector<Particle> particles;
  (*pf).getParticles(particles);

  // Compute the average pose and convert the particles pose to
  // ROS array
  std::vector<pose6D> m_poses;
  for (size_t i = 0; i < particles.size(); i++) {
    // Push back to the poses array
    pose6D m_pose = particles[i].pose;
    m_poses.push_back(m_pose);
  }
  average_pose = pose6D(m_poses);
  // Normalize average pose angles between [-pi,pi]
  average_pose.roll = normalizeAngle(average_pose.roll);
  // average_pose.pitch = cam_pitch + normalizeAngle(average_pose.pitch);
  average_pose.pitch = normalizeAngle(average_pose.pitch);
  average_pose.yaw   = normalizeAngle(average_pose.yaw);

#if MAP3D == 1
  // Save current sensor data to use in the next iteration
  // -------------------------------------------------------------------
  (*pf).p_feature_depths = feature_depths;
  // -------------------------------------------------------------------
  // Compute camera-world axis align matrix
  pose6D             align_pose(0., 0., cam_height, -PI / 2, 0., -PI / 2);
  std::vector<float> c2w_rot;
  align_pose.toRotMatrix(c2w_rot);
  // -------------------------------------------------------------------
  // Compute 3D position of set of features to use in the PF on the next
  // interation
  std::vector<float> Rot;
  average_pose.toRotMatrix(Rot);
  (*pf).p_features.clear();
  for (size_t i = 0; i < features.size(); i++) {
    // Compute depth of image feature
    int   x     = features[i].u;
    int   y     = features[i].v;
    int   idx   = x + img_width * y;
    float depth = feature_depths[idx];

    // Check validity of depth information
    if (!std::isfinite(feature_depths[idx]))
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
    Feature m_feature;
    m_feature.pos.x = point_cam.x * Rot[0] + point_cam.y * Rot[1] +
                      point_cam.z * Rot[2] + average_pose.x;
    m_feature.pos.y = point_cam.x * Rot[3] + point_cam.y * Rot[4] +
                      point_cam.z * Rot[5] + average_pose.y;
    m_feature.pos.z = point_cam.x * Rot[6] + point_cam.y * Rot[7] +
                      point_cam.z * Rot[8] + average_pose.z;
    // -------------------------------------------------------
    // Copy other fields
    m_feature.u    = features[i].u;
    m_feature.v    = features[i].v;
    m_feature.type = features[i].type;
    // Save it
    (*pf).p_features.push_back(m_feature);
  }
  // -------------------------------------------------------------------
#endif
}

pose6D Localizer::getPose() const { return average_pose; }

void Localizer::getParticles(std::vector<pose6D>& in) const
{
  // Get particles and resize input vector
  std::vector<Particle> particles;
  (*pf).getParticles(particles);
  in.resize(particles.size());

  for (size_t i = 0; i < in.size(); i++) in[i] = particles[i].pose;
}
