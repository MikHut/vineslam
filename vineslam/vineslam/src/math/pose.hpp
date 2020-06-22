#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#include "stat.hpp"

#include "point.hpp"

namespace vineslam
{
struct pose {
  // Default constructor
  pose()
  {
    x     = 0.;
    y     = 0.;
    z     = 0.;
    roll  = 0.;
    pitch = 0.;
    yaw   = 0.;
  }

  // Construct with 3D values
  pose(const float& x_,
       const float& y_,
       const float& z_,
       const float& roll_,
       const float& pitch_,
       const float& yaw_)
  {
    x     = x_;
    y     = y_;
    z     = z_;
    roll  = roll_;
    pitch = pitch_;
    yaw   = yaw_;
  }

  // Construct with 2D values
  pose(const float& x_, const float& y_, const float& yaw_)
  {
    x     = x_;
    y     = y_;
    z     = 0.;
    roll  = 0.;
    pitch = 0.;
    yaw   = yaw_;
  }

  // Construct with another pose
  pose(const pose& other)
  {
    x     = other.x;
    y     = other.y;
    z     = other.z;
    roll  = other.roll;
    pitch = other.pitch;
    yaw   = other.yaw;
    dist  = other.dist;
  }

  // Construct from set of poses
  pose(const std::vector<pose>& poses)
  {
    pose   mean(0., 0., 0., 0., 0., 0.);
    point  stdev(0., 0., 0.);
    double th = 0;

    // Calculate mean of all the robot poses
    for (const auto& pose : poses) mean = mean + pose;
    mean = mean / (float)poses.size();

    // Calculate standard deviation of all the robot positions
    for (const auto& pose : poses) {
      stdev.x += pow(pose.x - mean.x, 2);
      stdev.y += pow(pose.y - mean.y, 2);
    }
    stdev.x = sqrt(stdev.x / (float)poses.size());
    stdev.y = sqrt(stdev.y / (float)poses.size());

    // Save pose and gaussian distribution
    (*this) = mean;
    (*this).setDist(Gaussian<point, point>(point(mean.x, mean.y, mean.z), stdev));
  }

  // Pose from homogeneous transformation [R|t]
  pose(std::array<float, 9> Rot, std::array<float, 3> trans)
  {
    // -------------------------------------------
    // ------------ Rotation matrix to Euler angles
    // -------------------------------------------
    // - Check that pitch is not a singularity
    if (std::fabs(Rot[6]) >= 1.) {
      yaw = 0.;

      if (Rot[6] < 0.) {
        roll  = atan2(Rot[1], Rot[2]);
        pitch = PI / 2.;
      } else {
        roll  = atan2(-Rot[1], -Rot[2]);
        pitch = -PI / 2.;
      }
    } else {
      pitch = -asin(Rot[6]);
      roll  = atan2(Rot[7] / cos(pitch), Rot[8] / cos(pitch));
      yaw   = atan2(Rot[3] / cos(pitch), Rot[0] / cos(pitch));
    }
    // -------------------------------------------
    // ------------ Translation vector to x, y, z
    // -------------------------------------------
    x = trans[0];
    y = trans[1];
    z = trans[2];
  }

  // Assignment operator
  pose& operator=(const pose& other)
  {
    x     = other.x;
    y     = other.y;
    z     = other.z;
    roll  = other.roll;
    pitch = other.pitch;
    yaw   = other.yaw;

    dist = other.dist;

    return *this;
  }

  // Addition operator
  pose operator+(const pose& other) const
  {
    pose result(*this);
    result.x += other.x;
    result.y += other.y;
    result.z += other.z;
    result.roll += other.roll;
    result.pitch += other.pitch;
    result.yaw += other.yaw;

    return result;
  }

  // Subtraction operator
  pose operator-(const pose& other) const
  {
    pose result(*this);
    result.x -= other.x;
    result.y -= other.y;
    result.z -= other.z;
    result.roll -= other.roll;
    result.pitch -= other.pitch;
    result.yaw -= other.yaw;

    return result;
  }

  // Division by scalar operator
  pose operator/(const float& val) const
  {
    pose result(*this);
    result.x /= val;
    result.y /= val;
    result.z /= val;
    result.roll /= val;
    result.pitch /= val;
    result.yaw /= val;

    return result;
  }

  // Normalize pose angles
  void normalize()
  {
    roll = fmod(roll, 2 * PI);
    if (roll > PI)
      roll -= 2 * PI;

    pitch = fmod(pitch, 2 * PI);
    if (pitch > PI)
      pitch -= 2 * PI;

    yaw = fmod(yaw, 2 * PI);
    if (yaw > PI)
      yaw -= 2 * PI;
  }

  // Set gaussian noise characterizing the robot pose
  void setDist(const Gaussian<point, point>& other) { dist = other; }

  // Function to get the 3D point of the corresponding pose
  point getXYZ() const { return point(x, y, z); }

  // Function to get the gaussian distribution
  Gaussian<point, point> getDist() const { return dist; }

  // 3D euclidean distance
  float distance(const pose& other) const
  {
    float dist_x = x - other.x;
    float dist_y = y - other.y;
    float dist_z = z - other.z;
    return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
  }

  // 2D euclidean distance
  float distanceXY(const pose& other) const
  {
    float dist_x = x - other.x;
    float dist_y = y - other.y;
    return sqrt(dist_x * dist_x + dist_y * dist_y);
  }

  // Convert pose Euler angles to a Rotation matrix
  void toRotMatrix(std::array<float, 9>& m_rot) const
  {
    float ci = cos(roll);
    float cj = cos(pitch);
    float ch = cos(yaw);
    float si = sin(roll);
    float sj = sin(pitch);
    float sh = sin(yaw);
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

  // Convert pose to Eigen 6D pose
  Eigen::VectorXf toEig3D() const
  {
    Eigen::VectorXf vec(6, 1);
    vec << x, y, z, roll, pitch, yaw;
    return vec;
  }

  // Convert pose to Eigen 6D pose
  Eigen::VectorXf toEig2D() const
  {
    Eigen::VectorXf vec(3, 1);
    vec << x, y, yaw;
    return vec;
  }

  // Cartesian coordinates
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;

  // Gaussian uncertainty
  Gaussian<point, point> dist;
};

// stdout operator
static std::ostream& operator<<(std::ostream& out, pose const& p)
{
  return out << "Pose (" << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.roll << ' '
             << p.pitch << ' ' << p.yaw << ")\n"
             << "Distribution: (" << p.dist.stdev.x << ' ' << p.dist.stdev.y
             << ")\n";
}
}; // namespace vineslam
