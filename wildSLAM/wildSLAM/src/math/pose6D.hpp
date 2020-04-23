#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#include "ellipse2D.hpp"
#include "point3D.hpp"

class pose6D
{
public:
  // Default constructor
  pose6D()
  {
    x     = 0.;
    y     = 0.;
    z     = 0.;
    roll  = 0.;
    pitch = 0.;
    yaw   = 0.;
  }

  // Construct with values
  pose6D(const float& x_,
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

  // Construct with another pose6D
  pose6D(const pose6D& other)
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
  pose6D(const std::vector<pose6D>& poses)
  {
    pose6D    mean(0., 0., 0., 0., 0., 0.);
    ellipse2D gauss(0., 0., 0.);
    double    th = 0;

    // Calculate mean of all the robot poses
    for (size_t i = 0; i < poses.size(); i++) mean = mean + poses[i];
    mean = mean / (float)poses.size();

    // Calculate standard deviation of all the robot positions
    for (size_t i = 0; i < poses.size(); i++) {
      gauss.stdX += pow(poses[i].x - mean.x, 2);
      gauss.stdY += pow(poses[i].y - mean.y, 2);
    }
    gauss.stdX = sqrt(gauss.stdX / (float)poses.size());
    gauss.stdY = sqrt(gauss.stdY / (float)poses.size());
    th         = tan(mean.yaw);

    // Save pose and gaussian distribution
    (*this) = mean;
    (*this).setDist(gauss);
  }

  // Assignment operator
  pose6D operator=(const pose6D& other)
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
  pose6D operator+(const pose6D& other) const
  {
    pose6D result(*this);
    result.x += other.x;
    result.y += other.y;
    result.z += other.z;
    result.roll += other.roll;
    result.pitch += other.pitch;
    result.yaw += other.yaw;

    return result;
  }

  // Subtraction operator
  pose6D operator-(const pose6D& other) const
  {
    pose6D result(*this);
    result.x -= other.x;
    result.y -= other.y;
    result.z -= other.z;
    result.roll -= other.roll;
    result.pitch -= other.pitch;
    result.yaw -= other.yaw;

    return result;
  }

  // Division by scalar operator
  pose6D operator/(const float& val) const
  {
    pose6D result(*this);
    result.x /= val;
    result.y /= val;
    result.z /= val;
    result.roll /= val;
    result.pitch /= val;
    result.yaw /= val;

    return result;
  }

  // Set gaussian noise characterizing the robot pose
  void setDist(const ellipse2D& other) { dist = other; }

  // Function to get the 3D point of the corresponding pose
  point3D getXYZ() { return point3D(x, y, z); }

  // Function to get the gaussian distribution
  ellipse2D getDist() { return dist; }

  // 3D euclidean distance
  float distance(const pose6D& other) const
  {
    float dist_x = x - other.x;
    float dist_y = y - other.y;
    float dist_z = z - other.z;
    return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
  }

  // 2D euclidean distance
  float distanceXY(const pose6D& other) const
  {
    float dist_x = x - other.x;
    float dist_y = y - other.y;
    return sqrt(dist_x * dist_x + dist_y * dist_y);
  }

  // Convert pose Euler angles to a Rotation matrix
  void toRotMatrix(std::vector<float>& m_rot)
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

    // Resize vector to store rotation matrix
    m_rot.resize(9);

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
  Eigen::VectorXd toEig3D()
  {
    Eigen::VectorXd vec(6, 1);
    vec << x, y, z, roll, pitch, yaw;
    return vec;
  }

  // Convert pose to Eigen 6D pose
  Eigen::VectorXd toEig2D()
  {
    Eigen::VectorXd vec(3, 1);
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
  ellipse2D dist;

private:
};

// stdout operator
static std::ostream& operator<<(std::ostream& out, pose6D const& p)
{
  return out << "Pose (" << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.roll << ' '
             << p.pitch << ' ' << p.yaw << ")\n"
             << "Distribution: (" << p.dist.stdX << ' ' << p.dist.stdY << ' '
             << p.dist.TH << ")\n";
}
