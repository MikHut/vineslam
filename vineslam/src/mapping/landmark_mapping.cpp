#include "../../include/vineslam/mapping/landmark_mapping.hpp"

namespace vineslam
{
LandmarkMapper::LandmarkMapper(Parameters params) : params_(std::move(params))
{
  filter_frequency_ = 5;
  stdev_threshold_ = 0.2;

  // Initialize iterator
  it_ = 0;
}

void LandmarkMapper::init(const Pose& pose, const std::vector<float>& bearings, const std::vector<float>& depths,
                          const std::vector<int>& labels, OccupancyMap& grid_map)
{
  int n_obsv = bearings.size();
  Gaussian<Point, Point> robot_gauss = pose.getDist();

  // Convert 6-DOF pose to homogenous transformation
  std::array<float, 9> Rot{};
  Point trans = pose.getXYZ();
  pose.toRotMatrix(Rot);

  // Initialize landmark identifier
  id_ = 0;

  // Compute initial covariance matrix
  // - proportional to the pose signal and the distance to the robot
  for (int i = 0; i < n_obsv; i++)
  {
    // Construct the observations vector
    Eigen::VectorXf z(2, 1);
    z << depths[i], bearings[i];

    // Calculate
    // - the initial estimation of the landmark on map's referential frame
    float th = normalizeAngle(bearings[i]);
    Point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);
    Point X;
    X.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
    X.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
    X.z_ = 0.;

    // Push back a Kalman Filter object for the respective landmark
    KF kf(params_, X.toEig2D(), pose.toEig2D(), robot_gauss.stdev_.toEig2D(), z);
    filters.push_back(kf);

    // Insert the landmark on the map, with a single observation
    // and get the correspondent standard deviation
    id_++;
    Point pos(X.x_, X.y_, 0.);
    Gaussian<Point, Point> std = filters[filters.size() - 1].getStdev();
    grid_map.insert(SemanticFeature(pos, std, labels[i]), id_);
  }

  // Update iterator
  it_++;
}

void LandmarkMapper::process(const Pose& pose, const std::vector<SemanticFeature>& landmarks,
                             const std::vector<int>& labels, OccupancyMap& grid_map)
{
  // Compute local map on robot's referential frame
  std::vector<Point> l_map = base2map(pose, landmarks);
  // Convert local map to bearings and depths
  std::vector<float> bearings_(landmarks.size());
  std::vector<float> depths_(landmarks.size());
  for (size_t i = 0; i < l_map.size(); i++)
  {
    depths_[i] = std::sqrt(pow(l_map[i].x_, 2) + pow(l_map[i].y_, 2));
    bearings_[i] = std::atan2(l_map[i].y_, l_map[i].x_) - pose.Y_;
  }
  // Estimate global map
  predict(pose, bearings_, depths_, labels, grid_map);

  // Update iterator
  it_++;

  // Filter semantic map at a given frequency
  if (it_ % filter_frequency_ == 0)
    filter(grid_map);
}

std::vector<Point> LandmarkMapper::base2map(const Pose& pose, const std::vector<SemanticFeature>& landmarks)
{
  // Convert 6-DOF pose to homogenous transformation
  std::array<float, 9> Rot{};
  Point trans = pose.getXYZ();
  pose.toRotMatrix(Rot);

  std::vector<Point> landmarks_;
  for (const auto& landmark : landmarks)
  {
    Point X_cam(landmark.pos_.x_, landmark.pos_.y_, 0.);

    // Convert landmark to map's referential frame
    Point X_map;
    X_map.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
    X_map.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
    X_map.z_ = 0.;

    // Convert landmark to robot's referential frame and insert
    // on array of landmarks
    Point X_robot = X_map - pose.getXYZ();
    landmarks_.push_back(X_robot);
  }

  return landmarks_;
}

void LandmarkMapper::predict(const Pose& pose, const std::vector<float>& bearings, const std::vector<float>& depths,
                             const std::vector<int>& labels, OccupancyMap& grid_map)
{
  int n_obsv = bearings.size();
  Gaussian<Point, Point> robot_gauss = pose.getDist();

  // Convert 6DOF pose to homogenous transformation
  std::array<float, 9> Rot{};
  pose.toRotMatrix(Rot);
  Point trans = pose.getXYZ();

  for (int i = 0; i < n_obsv; i++)
  {
    // Calculate the landmark position on map's referential frame
    // based on the ith observation
    float th = normalizeAngle(bearings[i]);
    Point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);
    Point X;
    X.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2] + trans.x_;
    X.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5] + trans.y_;
    X.z_ = 0.;

    // Construct the observations vector
    VectorXf z(2, 1);
    z << depths[i], bearings[i];

    // Check if the landmark already exists in the map
    std::pair<int, Point> correspondence = findCorr(X, grid_map);
    // If not, initialize the landmark on the map, as well as the
    // correspondent Kalman Filter
    if (correspondence.first < 0)
    {
      Eigen::MatrixXd R(2, 2);

      // Initialize the Kalman Filter
      KF kf(params_, X.toEig2D(), pose.toEig2D(), robot_gauss.stdev_.toEig2D(), z);
      filters.push_back(kf);

      // Insert the landmark on the map, with a single observation
      id_++;
      Gaussian<Point, Point> gauss = filters[filters.size() - 1].getStdev();
      grid_map.insert(SemanticFeature(X, gauss, labels[i]), id_);
    }
    // If so, update the landmark position estimation using a Kalman
    // Filter call
    else
    {
      // Invocate the Kalman Filter
      filters[correspondence.first - 1].process(pose.toEig2D(), robot_gauss.stdev_.toEig2D(), z);
      // Get the state vector and the standard deviation associated
      // with the estimation
      Point X_out = filters[correspondence.first - 1].getState();
      Gaussian<Point, Point> gauss = filters[correspondence.first - 1].getStdev();

      // Update the estimation on the map
      grid_map.update(SemanticFeature(X_out, gauss, labels[i]), correspondence.first, correspondence.second.x_,
                      correspondence.second.y_);
    }
  }
}

std::pair<int, Point> LandmarkMapper::findCorr(const Point& pos, OccupancyMap& grid_map)
{
  int best_correspondence = -1;
  float best_aprox = 0.5;

  Point correspondence;

  // Search on current cell first
  for (const auto& m_landmark : grid_map(pos.x_, pos.y_, 0).landmarks_)
  {
    float dist = pos.distanceXY(m_landmark.second.pos_);

    if (dist < best_aprox)
    {
      correspondence = m_landmark.second.pos_;
      best_correspondence = m_landmark.first;
      best_aprox = dist;
    }
  }

  // Search on adjacent cells then
  int number_layers = (best_correspondence == -1) ? 2 : 1;
  std::vector<Cell> adjacents;
  grid_map.getAdjacent(pos.x_, pos.y_, 0., number_layers, adjacents);
  for (const auto& m_cell : adjacents)
  {
    for (const auto& m_landmark : m_cell.landmarks_)
    {
      float dist = pos.distanceXY(m_landmark.second.pos_);
      if (dist < best_aprox)
      {
        correspondence = m_landmark.second.pos_;
        best_correspondence = m_landmark.first;
        best_aprox = dist;
      }
    }
  }

  return std::pair<int, Point>(best_correspondence, correspondence);
}

void LandmarkMapper::localMap(const std::vector<float>& bearings, const std::vector<float>& depths,
                              std::vector<SemanticFeature>& landmarks) const
{
  Pose pitch_comp(0., 0., 0., 0., -cam_pitch_, 0.);
  std::array<float, 9> Rot{};
  pitch_comp.toRotMatrix(Rot);

  landmarks.resize(bearings.size());
  for (size_t i = 0; i < bearings.size(); i++)
  {
    // Calculate the estimation of the landmark position on
    // camera's referential frame
    float th = normalizeAngle(bearings[i]);
    Point X_cam(depths[i] * std::cos(th), depths[i] * std::sin(th), 0.);

    // Compensate camera inclination
    Point X_robot;
    X_robot.x_ = X_cam.x_ * Rot[0] + X_cam.y_ * Rot[1] + X_cam.z_ * Rot[2];
    X_robot.y_ = X_cam.x_ * Rot[3] + X_cam.y_ * Rot[4] + X_cam.z_ * Rot[5];
    X_robot.z_ = X_cam.z_;

    // Insert on output struct
    landmarks[i].pos_ = X_robot;
  }
}

void LandmarkMapper::filter(OccupancyMap& grid_map) const
{
  int old_limit = grid_map(0).n_landmarks_ - (grid_map(0).n_landmarks_ / 10);

  for (auto& cell : grid_map(0))
  {
    std::map<int, SemanticFeature> m_landmarks = cell.landmarks_;
    for (const auto& m_landmark : m_landmarks)
    {
      if (m_landmark.first < old_limit && (m_landmark.second.gauss_.stdev_.x_ > stdev_threshold_ ||
                                           m_landmark.second.gauss_.stdev_.y_ > stdev_threshold_))
      {
        cell.landmarks_.erase(m_landmark.first);
      }
    }
  }
}

}  // namespace vineslam
