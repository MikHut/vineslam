#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>

#include <math/point.hpp>
#include <math/pose.hpp>
#include <math/stat.hpp>
#include <occupancy_map.hpp>
#include <feature.hpp>

namespace vineslam
{

class ICP
{
public:
  // Class contructor:
  // - sets the default stop criteria parameters
  ICP(const std::string& config_path);

  // ICP main routine - aligns two point clouds
  bool align(const std::array<float, 9>& m_R,
             const std::array<float, 3>& m_t,
             float&                      rms_error,
             std::vector<ImageFeature>&  aligned);
  bool align(float& rms_error, std::vector<ImageFeature>& aligned);

  // Compute the rms error of the alignment between a source and the current target
  // cloud considering a specific pose
  bool score(const pose&                      spose,
             const std::vector<ImageFeature>& scloud,
             float&                           rms_error);

  // Methods to set the stop criteria parameters and inliers consideration
  void setMaxIterations(const int& m_max_iters) { max_iters = m_max_iters; }
  void setTolerance(const float& m_tolerance) { tolerance = m_tolerance; }
  void setThreshold(const float& m_threshold) { dist_threshold = m_threshold; }

  // Methods to receive the source and target point clouds
  void setInputTarget(const OccupancyMap& m_target)
  {
    target = new OccupancyMap(m_target);
  }
  void setInputSource(const std::vector<ImageFeature>& m_source)
  {
    source = m_source;
  }

  // Method to get the computed transformation between the source and target
  void getTransform(std::array<float, 9>& m_R, std::array<float, 3>& m_t) const
  {
    m_R = R;
    m_t = t;
  }

  // Method to export the Gaussian distributions
  void getProb(Gaussian<float, float>& m_sprob,
               Gaussian<float, float>& m_dprob) const
  {
    m_sprob = sprob;
    m_dprob = dprob;
  }

  // Methods to export the errors arrays
  void getErrors(std::vector<float>& serror, std::vector<float>& derror) const
  {
    serror = serrorvec;
    derror = derrorvec;
  }

private:
  // Method that performs a single ICP step
  bool step(Eigen::Matrix3f& m_R, Eigen::Vector3f& m_t, float& rms_error);

  // Auxiliar functions to convert from std arrays to eigen and vice versa
  inline void stdToEig(const std::array<float, 9>& m_R, Eigen::Matrix3f& Rot);
  inline void stdToEig(const std::array<float, 3>& m_t, Eigen::Vector3f& trans);
  inline void eigToStd(const Eigen::Matrix3f& Rot, std::array<float, 9>& m_R);
  inline void eigToStd(const Eigen::Vector3f& trans, std::array<float, 3>& m_t);

  // Parameters:
  // - maximum number of iterations
  int max_iters;
  // - minimum distance between iterations
  float tolerance;
  // - Maximum distance value between features to consider as correspondence inlier
  float dist_threshold;
  // - Boolean to choose if we reject or not outliers
  bool reject_outliers;

  // Source and target point clouds
  OccupancyMap*             target;
  std::vector<ImageFeature> source;

  // Last homogeneous transformation computed
  std::array<float, 9> R;
  std::array<float, 3> t;

  // Structure to store the correspondence errors resulting from the scan match
  std::vector<float> serrorvec;
  std::vector<float> derrorvec;

  // Probabilistic setup of the last calculations
  // - Gaussian representing the spatial alignment
  // - Gaussian representing the feature descriptor matching (if using image
  // descriptors)
  Gaussian<float, float> sprob; // spatial Gaussian
  Gaussian<float, float> dprob; // descriptor Gaussian
};

}; // namespace vineslam
