#pragma once

// Classes
#include <vineslam/params.hpp>
#include <vineslam/feature/semantic.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Stat.hpp>
#include <vineslam/math/Const.hpp>
#include <vineslam/mapping/landmark_ekf.hpp>

// std, eigen
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <numeric>

namespace vineslam
{
class LandmarkMapper
{
public:
  // Class constructor
  // - Loads the parameters
  explicit LandmarkMapper(Parameters params);

  // Global function that handles all the mapping process
  void process(const Pose& pose, const std::vector<SemanticFeature>& landmarks, const std::vector<int>& labels,
               OccupancyMap& grid_map);

  // Initializes the map
  // - Invocated only once to insert the first observations on the map
  void init(const Pose& pose, const std::vector<float>& bearings, const std::vector<float>& pitches,
            const std::vector<float>& depths, const std::vector<int>& labels, OccupancyMap& grid_map);

  // Computes a local map on camera's referential given a set of range-bearing
  // observations
  void localMap(const Pose& cam_origin_pose, const std::vector<int>& labels, const std::vector<float>& bearings,
                const std::vector<float>& pitches, std::vector<SemanticFeature>& landmarks, OccupancyMap& grid_map,
                Pose robot_pose) const;

private:
  // Input parameters
  int filter_frequency_;
  float stdev_threshold_;
  Parameters params_;

  // Semantic Feature identifier
  int id_{};

  // Iteration number
  int it_;

  // Array of Kalman Filters, one for each landmark
  std::vector<KF> filters;

  // Estimates landmark positions based on the current observations
  void predict(const Pose& pose, const std::vector<float>& bearings, const std::vector<float>& pitches,
               const std::vector<float>& depths, const std::vector<int>& labels, OccupancyMap& grid_map);

  // Filters semantic map based on the mapping uncertainty
  // - old landmarks that have high uncertainty are removed
  void filter(OccupancyMap& grid_map) const;

  // Searches from correspondences between observations and landmarks
  // already mapped
  static std::pair<int, SemanticFeature> findCorr(const Point& l_pos, const int& label, OccupancyMap& grid_map);
};

}  // namespace vineslam
