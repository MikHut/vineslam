#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#include "Stat.hpp"
#include "Point.hpp"
#include "Const.hpp"

namespace vineslam
{
struct Pose
{
  // Default constructor
  Pose()
  {
    x_ = 0.;
    y_ = 0.;
    z_ = 0.;
    R_ = 0.;
    P_ = 0.;
    Y_ = 0.;
  }

  // Construct with 3D values
  Pose(const float& x, const float& y, const float& z, const float& R, const float& P, const float& Y)
  {
    x_ = x;
    y_ = y;
    z_ = z;
    R_ = R;
    P_ = P;
    Y_ = Y;
  }

  // Construct with 2D values
  Pose(const float& x, const float& y, const float& Y)
  {
    x_ = x;
    y_ = y;
    z_ = 0.;
    R_ = 0.;
    P_ = 0.;
    Y_ = Y;
  }

  // Construct with another pose
  Pose(const Pose& other)
  {
    x_ = other.x_;
    y_ = other.y_;
    z_ = other.z_;
    R_ = other.R_;
    P_ = other.P_;
    Y_ = other.Y_;
    gaussian_dist_ = other.gaussian_dist_;
  }

  // Construct from set of poses
  explicit Pose(const std::vector<Pose>& poses)
  {
    Pose mean(0., 0., 0., 0., 0., 0.);
    Pose mean_cos(0., 0., 0., 0., 0., 0.);
    Pose mean_sin(0., 0., 0., 0., 0., 0.);
    Point stdev(0., 0., 0.);

    auto n_poses = static_cast<float>(poses.size());

    // Calculate mean of all the robot poses
    // - for the orientations: Mean of circular quantities
    //  (https://en.wikipedia.org/wiki/Mean_of_circular_quantities)
    for (const auto& pose : poses)
    {
      mean.x_ += pose.x_;
      mean.y_ += pose.y_;
      mean.z_ += pose.z_;
      mean_cos.R_ += std::cos(pose.R_);
      mean_sin.R_ += std::sin(pose.R_);
      mean_cos.P_ += std::cos(pose.P_);
      mean_sin.P_ += std::sin(pose.P_);
      mean_cos.Y_ += std::cos(pose.Y_);
      mean_sin.Y_ += std::sin(pose.Y_);
    }
    mean.x_ /= n_poses;
    mean.y_ /= n_poses;
    mean.z_ /= n_poses;
    mean.R_ = std::atan2(mean_sin.R_ / n_poses, mean_cos.R_ / n_poses);
    mean.P_ = std::atan2(mean_sin.P_ / n_poses, mean_cos.P_ / n_poses);
    mean.Y_ = std::atan2(mean_sin.Y_ / n_poses, mean_cos.Y_ / n_poses);

    // Calculate standard deviation of all the robot positions
    for (const auto& pose : poses)
    {
      stdev.x_ += std::pow(pose.x_ - mean.x_, 2);
      stdev.y_ += std::pow(pose.y_ - mean.y_, 2);
      stdev.z_ += std::pow(pose.z_ - mean.z_, 2);
    }
    stdev.x_ = std::sqrt(stdev.x_ / static_cast<float>(poses.size()));
    stdev.y_ = std::sqrt(stdev.y_ / static_cast<float>(poses.size()));
    stdev.z_ = std::sqrt(stdev.z_ / static_cast<float>(poses.size()));

    // Save pose and gaussian distribution
    (*this) = mean;
    (*this).setDist(Gaussian<Point, Point>(Point(mean.x_, mean.y_, mean.z_), stdev));
  }

  // Pose from homogeneous transformation [R|t]
  Pose(std::array<float, 9> Rot, std::array<float, 3> trans)
  {
    // -------------------------------------------
    // ------------ Rotation matrix to Euler angles
    // -------------------------------------------
    // - Check that pitch is not a singularity
    if (std::fabs(Rot[6]) >= 1.)
    {
      Y_ = 0.;

      if (Rot[6] < 0.)
      {
        R_ = std::atan2(Rot[1], Rot[2]);
        P_ = M_PI / 2.;
      }
      else
      {
        R_ = std::atan2(-Rot[1], -Rot[2]);
        P_ = -M_PI / 2.;
      }
    }
    else
    {
      P_ = -std::asin(Rot[6]);
      R_ = std::atan2(Rot[7] / std::cos(P_), Rot[8] / std::cos(P_));
      Y_ = std::atan2(Rot[3] / std::cos(P_), Rot[0] / std::cos(P_));
    }
    // -------------------------------------------
    // ------------ Translation vector to x, y, z
    // -------------------------------------------
    x_ = trans[0];
    y_ = trans[1];
    z_ = trans[2];
  }

  // Assignment operator
  Pose& operator=(const Pose& other)
  {
    x_ = other.x_;
    y_ = other.y_;
    z_ = other.z_;
    R_ = other.R_;
    P_ = other.P_;
    Y_ = other.Y_;

    gaussian_dist_ = other.gaussian_dist_;

    return *this;
  }

  // Addition operator
  Pose operator+(const Pose& other) const
  {
    Pose result(*this);
    result.x_ += other.x_;
    result.y_ += other.y_;
    result.z_ += other.z_;
    result.R_ += other.R_;
    result.P_ += other.P_;
    result.Y_ += other.Y_;

    return result;
  }

  // Subtraction operator
  Pose operator-(const Pose& other) const
  {
    Pose result(*this);
    result.x_ -= other.x_;
    result.y_ -= other.y_;
    result.z_ -= other.z_;
    result.R_ -= other.R_;
    result.P_ -= other.P_;
    result.Y_ -= other.Y_;

    return result;
  }

  // Division by scalar operator
  Pose operator/(const float& val) const
  {
    Pose result(*this);
    result.x_ /= val;
    result.y_ /= val;
    result.z_ /= val;
    result.R_ /= val;
    result.P_ /= val;
    result.Y_ /= val;

    return result;
  }

  // Normalize pose angles
  void normalize()
  {
    R_ = std::atan2(std::sin(R_), std::cos(R_));
    P_ = std::atan2(std::sin(P_), std::cos(P_));
    Y_ = std::atan2(std::sin(Y_), std::cos(Y_));
  }

  // 2D and 3D norms of the pose
  float norm2D() const
  {
    return std::sqrt(x_ * x_ + y_ * y_);
  }
  float norm3D() const
  {
    return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
  }

  // Set gaussian noise characterizing the robot pose
  void setDist(const Gaussian<Point, Point>& other)
  {
    gaussian_dist_ = other;
  }

  // Function to get the 3D point of the corresponding pose
  Point getXYZ() const
  {
    return Point(x_, y_, z_);
  }

  // Function to get the gaussian distribution
  void getDist(Gaussian<Point, Point>& dist) const
  {
    dist = gaussian_dist_;
  }

  // 3D euclidean distance
  float distance(const Pose& other) const
  {
    float dist_x = x_ - other.x_;
    float dist_y = y_ - other.y_;
    float dist_z = z_ - other.z_;
    return std::sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
  }

  // 2D euclidean distance
  float distanceXY(const Pose& other) const
  {
    float dist_x = x_ - other.x_;
    float dist_y = y_ - other.y_;
    return std::sqrt(dist_x * dist_x + dist_y * dist_y);
  }

  // Convert pose Euler angles to a Rotation matrix
  void toRotMatrix(std::array<float, 9>& m_rot) const
  {
    float ci = std::cos(R_);
    float cj = std::cos(P_);
    float ch = std::cos(Y_);
    float si = std::sin(R_);
    float sj = std::sin(P_);
    float sh = std::sin(Y_);
    float cc = ci * ch;
    float cs = ci * sh;
    float sc = si * ch;
    float ss = si * sh;

    // Store the values
    m_rot[0] = cj * ch;
    m_rot[1] = sj * sc - cs;
    m_rot[2] = sj * cc + ss;
    m_rot[3] = cj * sh;
    m_rot[4] = sj * ss + cc;
    m_rot[5] = sj * cs - sc;
    m_rot[6] = -sj;
    m_rot[7] = cj * si;
    m_rot[8] = cj * ci;
  }

  // Convert pose to a transformation matrix
  Tf toTf()
  {
    std::array<float, 9> l_R;
    (*this).toRotMatrix(l_R);

    return Tf(l_R, {x_, y_, z_});
  }

  // Convert pose to Eigen 6D pose
  Eigen::VectorXf toEig3D() const
  {
    Eigen::VectorXf vec(6, 1);
    vec << x_, y_, z_, R_, P_, Y_;
    return vec;
  }

  // Convert pose to Eigen 6D pose
  Eigen::VectorXf toEig2D() const
  {
    Eigen::VectorXf vec(3, 1);
    vec << x_, y_, Y_;
    return vec;
  }

  // Cartesian coordinates
  float x_{};
  float y_{};
  float z_{};
  float R_{};
  float P_{};
  float Y_{};

  // Gaussian uncertainty
  Gaussian<Point, Point> gaussian_dist_;
};

}  // namespace vineslam
