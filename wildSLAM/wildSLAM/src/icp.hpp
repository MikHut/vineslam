#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>

#include "math/point3D.hpp"
#include "math/pose6D.hpp"
#include "math/ellipse2D.hpp"
#include "occupancy_map.hpp"
#include "feature.hpp"

namespace wildSLAM
{
class ICP
{
public:
  // Class contructor:
  // - sets the default stop criteria parameters
  ICP();

  // ICP main routine - aligns two point clouds
  bool align(const std::array<float, 9>& m_R,
             const std::array<float, 3>& m_t,
             std::vector<Feature>&       aligned);
  bool align(std::vector<Feature>& aligned);

  // Methods to set the stop criteria parameters
  void setMaxIterations(const int& m_max_iters) { max_iters = m_max_iters; }
  void setTolerance(const float& m_tolerance) { tolerance = m_tolerance; }

  // Methods to receive the source and target point clouds
  void setInputTarget(const OccupancyMap& m_target)
  {
    target = new OccupancyMap(m_target);
  }
  void setInputSource(const std::vector<Feature>& m_source) { source = m_source; }

  // Method to get the computed transformation between the source and target
  void getTransform(std::array<float, 9>& m_R, std::array<float, 3>& m_t) const
  {
    m_R = R;
    m_t = t;
  }

  // private:
  // Method that performs a single ICP step
  bool step(Eigen::Matrix3f& m_R, Eigen::Vector3f& m_t, float& rms_error);

  // Auxiliar functions to convert from std arrays to eigen and vice versa
  void stdToEig(const std::array<float, 9>& m_R, Eigen::Matrix3f& Rot);
  void stdToEig(const std::array<float, 3>& m_t, Eigen::Vector3f& trans);
  void eigToStd(const Eigen::Matrix3f& Rot, std::array<float, 9>& m_R);
  void eigToStd(const Eigen::Vector3f& trans, std::array<float, 3>& m_t);
  //  inline void stdToEig(const std::array<float, 9>& m_R, Eigen::Matrix3f& Rot);
  //  inline void stdToEig(const std::array<float, 3>& m_t, Eigen::Vector3f& trans);
  //  inline void eigToStd(const Eigen::Matrix3f& Rot, std::array<float, 9>& m_R);
  //  inline void eigToStd(const Eigen::Vector3f& trans, std::array<float, 3>& m_t);

  // Stop criteria parameters:
  // - maximum number of iterations
  int max_iters;
  // - minimum distance between iterations
  float tolerance;

  // Source and target point clouds
  OccupancyMap*        target;
  std::vector<Feature> source;

  // Last homogeneous transformation computed
  std::array<float, 9> R;
  std::array<float, 3> t;
};
}; // namespace wildSLAM
