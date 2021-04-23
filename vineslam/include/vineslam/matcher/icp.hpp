#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../params.hpp"
#include "../feature/feature.hpp"
#include "../mapping/occupancy_map.hpp"
#include "../math/Point.hpp"
#include "../math/Pose.hpp"
#include "../math/Tf.hpp"
#include "../math/Stat.hpp"

namespace vineslam
{
template <class T>
class ICP
{
public:
  // -------------------------------------------------------------------------------
  // ----- Class constructor - sets the default stop criteria parameters
  // -------------------------------------------------------------------------------
  ICP()
  {
    // Set the default stop criteria parameters
    // - they can (and should) be overwritten from the outside call (!)
    max_iters_ = 20;
    tolerance_ = 1e-3;
    dist_threshold_ = 0.2;
    reject_outliers_ = false;

    // Initialize homogeneous transformation
    R_array_ = { 1., 0., 0., 0., 1., 0., 0., 0., 1. };
    t_array_ = { 0., 0., 0. };
  }

  // -------------------------------------------------------------------------------
  // ----- ICP main routine - aligns two point clouds
  // -------------------------------------------------------------------------------
  bool align(Tf tf, float& rms_error, std::vector<T>& aligned)
  {
    if (source_vec_.empty())
    {
#if VERBOSE == 1
      std::cout << "WARNING (ICP::align): source cloud empty. Returning first guess." << std::endl;
#endif
      return true;
    }

    // Initialize stop criteria parameters
    int n_iters = 0;
    float delta_dist = 1e6;
    // Initialize homogeneous transformation
    Eigen::Matrix3f Rot;
    Eigen::Vector3f trans;
    stdToEig(tf.R_array_, Rot);
    stdToEig(tf.t_array_, trans);

    // Perform first iteration and save the error
    float p_rms_error;
    step(Rot, trans, p_rms_error);
    n_iters++;

    // Boolean to check if we found a suitable solution
    bool found_solution = false;

    rms_error = 0.;
    while (n_iters < max_iters_ && delta_dist > tolerance_)
    {
      if (step(Rot, trans, rms_error))
      {
        delta_dist = std::fabs(rms_error - p_rms_error);
        p_rms_error = rms_error;

        found_solution = true;
      }

      n_iters++;
    }

    if (!found_solution)  // invalid iteration
    {
#if VERBOSE == 1
      std::cout << "WARNING ICP::align: Scan matcher failed - none valid iteration..." << std::endl;
#endif
      return false;
    }

    if (n_iters == max_iters_)
    {
#if VERBOSE == 1
      std::cout << "WANRING ICP::aling: Scan matcher failed - it did not converge..." << std::endl;
#endif
      return false;
    }

    // Save homogeneous transformation solution
    eigToStd(Rot, R_array_);
    eigToStd(trans, t_array_);

    // Check if ICP produced a big step. If so, invalid iteration
    Tf tf_res(R_array_, t_array_);
    Tf tf_delta = tf.inverse() * tf_res;
    Pose delta_p(tf_delta.R_array_, tf_delta.t_array_);
    //    if (std::fabs(delta_p.x_) > 0.5 || std::fabs(delta_p.y_) > 0.5 || std::fabs(delta_p.z_) > 0.5 ||
    //        std::fabs(delta_p.R_) > 0.85 || std::fabs(delta_p.P_) > 0.85 || std::fabs(delta_p.Y_) > 0.85)
    //    {
    //#if VERBOSE == 1
    //      std::cout << "WARNING ICP::align: Huge jump detected on ICP - considering "
    //                   "iteration as invalid..."
    //                << std::endl;
    //#endif
    //      return false;
    //    }

    // Compute aligned point cloud
    aligned.resize(source_vec_.size());
    for (size_t i = 0; i < aligned.size(); i++)
    {
      aligned[i] = source_vec_[i];

      Point spt = source_vec_[i].pos_;
      Point apt;
      apt.x_ = spt.x_ * R_array_[0] + spt.y_ * R_array_[1] + spt.z_ * R_array_[2] + t_array_[0];
      apt.y_ = spt.x_ * R_array_[3] + spt.y_ * R_array_[4] + spt.z_ * R_array_[5] + t_array_[1];
      apt.z_ = spt.x_ * R_array_[6] + spt.y_ * R_array_[7] + spt.z_ * R_array_[8] + t_array_[2];

      aligned[i].pos_ = apt;
    }

    return true;
  }
  bool align(float& rms_error, std::vector<T>& aligned)
  {
    // Set rotation to identity and translation to 0
    std::array<float, 9> m_R = { 1., 0., 0., 0., 1., 0., 0., 0., 1. };
    std::array<float, 3> m_t{};

    Tf tf(m_R, m_t);

    return align(tf, rms_error, aligned);
  }

  // -------------------------------------------------------------------------------
  // ----- Methods to set the stop criteria parameters and inliers consideration
  // -------------------------------------------------------------------------------
  void setMaxIterations(const int& m_max_iters)
  {
    max_iters_ = m_max_iters;
  }
  void setTolerance(const float& m_tolerance)
  {
    tolerance_ = m_tolerance;
  }
  void setThreshold(const float& m_threshold)
  {
    dist_threshold_ = m_threshold;
  }
  void setRejectOutliersFlag(const bool& m_ro)
  {
    reject_outliers_ = m_ro;
  }

  // -------------------------------------------------------------------------------
  // ----- Methods to receive the source and target point clouds
  // -------------------------------------------------------------------------------
  void setInputTarget(OccupancyMap* m_target)
  {
    target_ = m_target;
  }
  void setInputSource(const std::vector<T>& m_source)
  {
    source_vec_ = m_source;
  }

  // -------------------------------------------------------------------------------
  // ----- Method to get the computed transformation between the source and target
  // -------------------------------------------------------------------------------
  void getTransform(Tf& result) const
  {
    result = Tf(R_array_, t_array_);
  }

  // -------------------------------------------------------------------------------
  // ----- Methods to export the errors arrays
  // -------------------------------------------------------------------------------
  void getErrors(std::vector<float>& serror) const
  {
    serror = error_vec_;
  }

private:
  // -------------------------------------------------------------------------------
  // ----- Method that performs a single ICP step
  // -------------------------------------------------------------------------------
  bool step(Eigen::Matrix3f& m_R, Eigen::Vector3f& m_t, float& rms_error)
  {
    // Initialize source and target means
    Eigen::Vector3f target_mean(0., 0., 0.);
    Eigen::Vector3f source_mean(0., 0., 0.);

    // Declare target and source matrices to save the correspondences
    Eigen::Matrix3Xf target_pts;
    Eigen::Matrix3Xf source_pts;
    target_pts.resize(3, source_vec_.size());
    source_pts.resize(3, source_vec_.size());

    // Inlier correspondences
    Eigen::Matrix3Xf inliers_tpoints;
    Eigen::Matrix3Xf inliers_spoints;

    // Declare matrices to store the result
    Eigen::Matrix3f delta_R;
    Eigen::Vector3f delta_t;

    // Iterator that will count the number of valid iterations
    int32_t j = 0;
    // Iterator that will count the number inliers
    int32_t nsamples = 0;

    // Arrays to all the correspondences errors
    error_vec_.clear();  // clear error array

    for (const auto& m_feature : source_vec_)
    {
      // Convert feature into the target reference frame using current [R|t] solution
      Eigen::Vector3f fsource(m_feature.pos_.x_, m_feature.pos_.y_, m_feature.pos_.z_);
      Eigen::Vector3f ftransformed = m_R * fsource + m_t;

      // Find nearest neighbor of the current point
      T _ftarget;
      T _ftransformed = m_feature;
      _ftransformed.pos_ = Point(ftransformed(0, 0), ftransformed(1, 0), ftransformed(2, 0));
      // Save minimum distance found (usually for the descriptor)
      float dist = 1e6;
      if (!target_->findNearest(_ftransformed, _ftarget, dist))
      {
        continue;
      }

      // Save source and target points
      Eigen::Vector3f ftarget(_ftarget.pos_.x_, _ftarget.pos_.y_, _ftarget.pos_.z_);
      target_pts.block<3, 1>(0, j) = ftarget;
      source_pts.block<3, 1>(0, j) = ftransformed;

      // Remove (or not) outliers using a displacement threshold
      // ----------------------------------------------------------------------------
      if (dist < dist_threshold_ || !reject_outliers_)
      {
        nsamples = inliers_spoints.cols() + 1;

        inliers_tpoints.conservativeResize(3, nsamples);
        inliers_spoints.conservativeResize(3, nsamples);
        // Push back inliers
        inliers_tpoints.block<3, 1>(0, nsamples - 1) = target_pts.block<3, 1>(0, j);
        inliers_spoints.block<3, 1>(0, nsamples - 1) = source_pts.block<3, 1>(0, j);

        // Accumulate the results to then compute the pointwise mean
        target_mean += inliers_tpoints.block<3, 1>(0, nsamples - 1);
        source_mean += inliers_spoints.block<3, 1>(0, nsamples - 1);

        // Store correspondences errors just for inliers
        error_vec_.push_back(dist);
      }
      // ----------------------------------------------------------------------------

      // Valid iteration
      j++;
    }

    if (j == 0)  // Invalid iteration
    {
#if VERBOSE == 1
      std::cout << "WARNING (ICP::step): Invalid iteration - none correspondence found..." << std::endl;
#endif
      return false;
    }
    if (nsamples == 0)  // Invalid iteration
    {
#if VERBOSE == 1
      std::cout << "WARNING (ICP::step): Invalid iteration - none inlier found..." << std::endl;
#endif
      return false;
    }

    // Compute center of mass of source and target point clouds
    target_mean /= static_cast<float>(nsamples);
    source_mean /= static_cast<float>(nsamples);

    // Compute pointwise difference in relation to the center of mass of each point
    // cloud
    // ------------------------------------------------------------------------------
    Eigen::MatrixXf ones;
    ones.resize(1, nsamples);
    ones.setOnes();
    // ------------------------------------------------------------------------------
    Eigen::MatrixXf tdiff;
    tdiff = target_mean * ones;
    tdiff = inliers_tpoints - tdiff;
    // ------------------------------------------------------------------------------
    Eigen::MatrixXf sdiff;
    sdiff = source_mean * ones;
    sdiff = inliers_spoints - sdiff;
    // ------------------------------------------------------------------------------

    // Compute A matrix to input the Single Value Decomposition
    Eigen::Matrix3f A = tdiff * sdiff.transpose();

    // Perform SVD to extract the rotation matrix and the translation vector
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    delta_R = svd.matrixU() * svd.matrixV().transpose();
    delta_t = target_mean - delta_R * source_mean;

    // Compute RMS error between the target and the aligned cloud
    rms_error = 0.;
    for (int i = 0; i < nsamples; i++)
    {
      Eigen::Vector3f m_target_pt = inliers_tpoints.block<3, 1>(0, i);
      Eigen::Vector3f m_source_pt = inliers_spoints.block<3, 1>(0, i);

      Eigen::Vector3f m_diff = m_target_pt - (delta_R * m_source_pt + delta_t);
      rms_error += m_diff.norm();
    }
    rms_error /= static_cast<float>(nsamples);

    // Compose the transformations
    m_R = delta_R * m_R;
    m_t = delta_R * m_t + delta_t;

    return true;
  }

  // -------------------------------------------------------------------------------
  // ----- Auxiliar functions to convert from std arrays to eigen and vice versa
  // -------------------------------------------------------------------------------
  static inline void stdToEig(const std::array<float, 9>& m_R, Eigen::Matrix3f& Rot)
  {
    Rot(0, 0) = m_R[0];
    Rot(0, 1) = m_R[1];
    Rot(0, 2) = m_R[2];
    Rot(1, 0) = m_R[3];
    Rot(1, 1) = m_R[4];
    Rot(1, 2) = m_R[5];
    Rot(2, 0) = m_R[6];
    Rot(2, 1) = m_R[7];
    Rot(2, 2) = m_R[8];
  }
  static inline void stdToEig(const std::array<float, 3>& m_t, Eigen::Vector3f& trans)
  {
    trans(0, 0) = m_t[0];
    trans(1, 0) = m_t[1];
    trans(2, 0) = m_t[2];
  }
  static inline void eigToStd(const Eigen::Matrix3f& Rot, std::array<float, 9>& m_R)
  {
    m_R[0] = Rot(0, 0);
    m_R[1] = Rot(0, 1);
    m_R[2] = Rot(0, 2);
    m_R[3] = Rot(1, 0);
    m_R[4] = Rot(1, 1);
    m_R[5] = Rot(1, 2);
    m_R[6] = Rot(2, 0);
    m_R[7] = Rot(2, 1);
    m_R[8] = Rot(2, 2);
  }
  static inline void eigToStd(const Eigen::Vector3f& trans, std::array<float, 3>& m_t)
  {
    m_t[0] = trans(0, 0);
    m_t[1] = trans(1, 0);
    m_t[2] = trans(2, 0);
  }

  // Parameters:
  // - maximum number of iterations
  int max_iters_;
  // - minimum distance between iterations
  float tolerance_;
  // - Maximum distance value between features to consider as correspondence inlier
  float dist_threshold_;
  // - Boolean to choose if we reject or not outliers
  bool reject_outliers_;

  // Source and target point clouds
  OccupancyMap* target_{};
  std::vector<T> source_vec_;

  // Last homogeneous transformation computed
  std::array<float, 9> R_array_{};
  std::array<float, 3> t_array_{};

  // Structure to store the correspondence errors resulting from the scan match
  std::vector<float> error_vec_;
};

}  // namespace vineslam
