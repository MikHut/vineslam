#include "mapper2D.hpp"

namespace vineslam
{

Mapper2D::Mapper2D(const std::string& config_path)
    : config_path(config_path)
{
  YAML::Node config = YAML::LoadFile(config_path);
  fx                = config["camera_info"]["fx"].as<float>();
  baseline          = config["camera_info"]["baseline"].as<float>();
  delta_d           = config["camera_info"]["delta_d"].as<float>();
}

void Mapper2D::init(const pose&               pose,
                    const std::vector<float>& bearings,
                    const std::vector<float>& depths,
                    const std::vector<int>&   labels,
                    OccupancyMap&             grid_map)
{
  int                    n_obsv      = bearings.size();
  Gaussian<point, point> robot_gauss = pose.getDist();

  // Convert 6-DOF pose to homogenous transformation
  std::array<float, 9> Rot   = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
  point                trans = pose.getXYZ();
  pose.toRotMatrix(Rot);

  // Initialize landmark identifier
  id = 0;

  // Compute initial covariance matrix
  // - proportional to the pose signal and the distance to the robot
  for (int i = 0; i < n_obsv; i++) {
    // Construct the observations vector
    Eigen::VectorXf z(2, 1);
    z << depths[i], bearings[i];

    // Calculate
    // - the initial estimation of the landmark on map's referential frame
    float th = normalizeAngle(bearings[i]);
    point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);
    point X;
    X.x = X_cam.x * Rot[0] + X_cam.y * Rot[1] + X_cam.z * Rot[2] + trans.x;
    X.y = X_cam.x * Rot[3] + X_cam.y * Rot[4] + X_cam.z * Rot[5] + trans.y;
    X.z = 0.;

    // Push back a Kalman Filter object for the respective landmark
    KF kf(X.toEig2D(), pose.toEig2D(), robot_gauss.stdev.toEig2D(), z, config_path);
    filters.push_back(kf);

    // Insert the landmark on the map, with a single observation
    // and get the correspondent standard deviation
    id++;
    point                  pos(X.x, X.y, 0.);
    Gaussian<point, point> std = filters[filters.size() - 1].getStdev();
    grid_map.insert(SemanticFeature(pos, std, labels[i]), id);
  }
}

void Mapper2D::process(const pose&               pose,
                       const std::vector<float>& bearings,
                       const std::vector<float>& depths,
                       const std::vector<int>&   labels,
                       OccupancyMap&             grid_map)
{
  // Compute local map on robot's referential frame
  std::vector<point> l_map = cam2base(pose, bearings, depths);
  // Convert local map to bearings and depths
  std::vector<float> bearings_(bearings.size());
  std::vector<float> depths_(depths.size());
  for (size_t i = 0; i < l_map.size(); i++) {
    depths_[i]   = std::sqrt(pow(l_map[i].x, 2) + pow(l_map[i].y, 2));
    bearings_[i] = std::atan2(l_map[i].y, l_map[i].x) - pose.yaw;
  }
  // Estimate global map
  predict(pose, bearings_, depths_, labels, grid_map);
}

std::vector<point> Mapper2D::cam2base(const pose&               pose,
                                      const std::vector<float>& bearings,
                                      const std::vector<float>& depths)
{
  std::vector<point> landmarks;

  // Convert 6-DOF pose to homogenous transformation
  std::array<float, 9> Rot   = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
  point                trans = pose.getXYZ();
  pose.toRotMatrix(Rot);

  for (size_t i = 0; i < bearings.size(); i++) {
    // Calculate the estimation of the landmark position on
    // camera's referential frame
    float th = normalizeAngle(bearings[i]);
    point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);

    // Convert landmark to map's referential frame
    point X_map;
    X_map.x = X_cam.x * Rot[0] + X_cam.y * Rot[1] + X_cam.z * Rot[2] + trans.x;
    X_map.y = X_cam.x * Rot[3] + X_cam.y * Rot[4] + X_cam.z * Rot[5] + trans.y;
    X_map.z = 0.;

    // Convert landmark to robot's referential frame and insert
    // on array of landmarks
    point X_robot = X_map - pose.getXYZ();
    landmarks.push_back(X_robot);
  }

  return landmarks;
}

void Mapper2D::predict(const pose&               pose,
                       const std::vector<float>& bearings,
                       const std::vector<float>& depths,
                       const std::vector<int>&   labels,
                       OccupancyMap&             grid_map)
{
  int                    n_obsv      = bearings.size();
  Gaussian<point, point> robot_gauss = pose.getDist();

  // Convert 6DOF pose to homogenous transformation
  std::array<float, 9> Rot = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
  pose.toRotMatrix(Rot);
  point trans = pose.getXYZ();

  for (int i = 0; i < n_obsv; i++) {
    // Calculate the landmark position on map's referential frame
    // based on the ith observation
    float th = normalizeAngle(bearings[i]);
    point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);
    point X;
    X.x = X_cam.x * Rot[0] + X_cam.y * Rot[1] + X_cam.z * Rot[2] + trans.x;
    X.y = X_cam.x * Rot[3] + X_cam.y * Rot[4] + X_cam.z * Rot[5] + trans.y;
    X.z = 0.;

    // Construct the observations vector
    VectorXf z(2, 1);
    z << depths[i], bearings[i];

    // Check if the landmark already exists in the map
    std::pair<int, point> correspondence = findCorr(X, grid_map);
    // If not, initialize the landmark on the map, as well as the
    // correspondent Kalman Filter
    if (correspondence.first < 0) {
      Eigen::MatrixXd R(2, 2);

      // Initialize the Kalman Filter
      KF kf(
          X.toEig2D(), pose.toEig2D(), robot_gauss.stdev.toEig2D(), z, config_path);
      filters.push_back(kf);

      // Insert the landmark on the map, with a single observation
      id++;
      Gaussian<point, point> gauss = filters[filters.size() - 1].getStdev();
      grid_map.insert(SemanticFeature(X, gauss, labels[i]), id);
    }
    // If so, update the landmark position estimation using a Kalman
    // Filter call
    else {
      // Invocate the Kalman Filter
      filters[correspondence.first - 1].process(
          pose.toEig2D(), robot_gauss.stdev.toEig2D(), z);
      // Get the state vector and the standard deviation associated
      // with the estimation
      point                  X_out = filters[correspondence.first - 1].getState();
      Gaussian<point, point> gauss = filters[correspondence.first - 1].getStdev();

      // Update the estimation on the map
      grid_map.update(SemanticFeature(X_out, gauss, labels[i]),
                      correspondence.first,
                      correspondence.second.x,
                      correspondence.second.y);
    }
  }
}

std::pair<int, point> Mapper2D::findCorr(const point& pos, OccupancyMap& grid_map)
{
  int   best_correspondence = -1;
  float best_aprox          = 0.5;

  point correspondence;

  // Search on current cell first
  for (const auto& m_landmark : grid_map(pos.x, pos.y).landmarks) {
    float dist = pos.distanceXY(m_landmark.second.pos);

    if (dist < best_aprox) {
      correspondence      = m_landmark.second.pos;
      best_correspondence = m_landmark.first;
      best_aprox          = dist;
    }
  }

  // Search on adjacent cells then
  int               number_layers = (best_correspondence == -1) ? 2 : 1;
  std::vector<Cell> adjacents;
  grid_map.getAdjacent(pos.x, pos.y, number_layers, adjacents);
  for (const auto& m_cell : adjacents) {
    for (const auto& m_landmark : m_cell.landmarks) {
      float dist = pos.distanceXY(m_landmark.second.pos);
      if (dist < best_aprox) {
        correspondence      = m_landmark.second.pos;
        best_correspondence = m_landmark.first;
        best_aprox          = dist;
      }
    }
  }

  return std::pair<int, point>(best_correspondence, correspondence);
}

void Mapper2D::localMap(const std::vector<float>&     bearings,
                        const std::vector<float>&     depths,
                        std::vector<SemanticFeature>& landmarks)
{
  landmarks.resize(bearings.size());
  for (size_t i = 0; i < bearings.size(); i++) {
    // Calculate the estimation of the landmark position on
    // camera's referential frame
    float th = normalizeAngle(bearings[i]);
    point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);

    // Insert on output struct
    landmarks[i].pos = X_cam;
  }
}

} // namespace vineslam
