#pragma once

// Classes
#include <params.hpp>
#include <kf.hpp>
#include <feature.hpp>
#include <occupancy_map.hpp>
#include <math/stat.hpp>
#include <math/point.hpp>
#include <math/pose.hpp>
#include <math/const.hpp>

// std, eigen
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <numeric>

namespace vineslam
{

class Mapper2D
{
public:
  // Class constructor
  // - Loads the parameters
  explicit Mapper2D(Parameters params);

  // Global function that handles all the mapping process
  void process(const pose&                         pose,
               const std::vector<SemanticFeature>& landmarks,
               const std::vector<int>&             labels,
               MapLayer&                       grid_map);

  // Initializes the map
  // - Invocated only once to insert the first observations on the map
  void init(const pose&               pose,
            const std::vector<float>& bearings,
            const std::vector<float>& depths,
            const std::vector<int>&   labels,
            MapLayer&             grid_map);

  // Computes a local map on camera's referential given a set of range-bearing
  // observations
  void localMap(const std::vector<float>&     bearings,
                const std::vector<float>&     depths,
                std::vector<SemanticFeature>& landmarks) const;

  // Setters
  void setCamPitch(const float& pitch) { cam_pitch = pitch; }

private:
  // Input parameters
  int        filter_frequency;
  float      stdev_threshold;
  float      baseline;
  float      cam_pitch{};
  float      fx;
  Parameters params;

  // Semantic Feature identifier
  int id{};

  // Iteration number
  int it;

  // Array of Kalman Filters, one for each landmark
  std::vector<KF> filters;

  // Estimates landmark positions based on the current observations
  void predict(const pose&               pose,
               const std::vector<float>& bearings,
               const std::vector<float>& depths,
               const std::vector<int>&   labels,
               MapLayer&             grid_map);

  // Filters semantic map based on the mapping uncertainty
  // - old landmarks that have high uncertainty are removed
  void filter(MapLayer& grid_map) const;

  // Computes a local map, on robot's frame
  static std::vector<point> base2map(const pose&                         pose,
                                     const std::vector<SemanticFeature>& landmarks);

  // Searches from correspondences between observations and landmarks
  // already mapped
  static std::pair<int, point> findCorr(const point& l_pos, MapLayer& grid_map);
};

} // namespace vineslam
