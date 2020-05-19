#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#define PI 3.14159265359

namespace wildSLAM
{
class point3D
{
public:
  // Default constructor
  point3D()
  {
    x = 0.;
    y = 0.;
    z = 0.;
  }

  // Construct with values
  point3D(const float& x_, const float& y_, const float& z_)
  {
    x = x_;
    y = y_;
    z = z_;
  }

  // Construct with another point3D
  point3D(const point3D& other)
  {
    x = other.x;
    y = other.y;
    z = other.z;
  }

  // Compute a 3D point from a 2D point, its depth, the camera parameters,
  // and an homogeneous transformation
  point3D(const int&                u,
          const int&                v,
          const float&              depth,
          const float&              cam_height,
          const float&              fx,
          const float&              fy,
          const float&              cx,
          const float&              cy,
          const std::vector<float>& Rot,
          const point3D&            trans)
  {
    // Compute camera-world axis align matrix
    std::array<float, 9> c2w_rot   = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    std::array<float, 3> c2w_trans = {0., 0., cam_height};

    // Project 2D feature into 3D world point in
    // camera's referential frame
    point3D point;
    point.x = (float)((u - cx) * (depth / fx));
    point.y = (float)((v - cy) * (depth / fy));
    point.z = depth;

    // Camera to map point cloud conversion
    // -------------------------------------------------------
    // Align world and camera axis
    point3D point_cam;
    point_cam.x = c2w_rot[0] * point.x + c2w_rot[1] * point.y +
                  c2w_rot[2] * point.z + c2w_trans[0];
    point_cam.y = c2w_rot[3] * point.x + c2w_rot[4] * point.y +
                  c2w_rot[5] * point.z + c2w_trans[1];
    point_cam.z = c2w_rot[6] * point.x + c2w_rot[7] * point.y +
                  c2w_rot[8] * point.z + c2w_trans[2];
    // -------------------------------------------------------
    // Apply robot pose to convert points to map's referential
    // frame
    x = point_cam.x * Rot[0] + point_cam.y * Rot[1] + point_cam.z * Rot[2] + trans.x;
    y = point_cam.x * Rot[3] + point_cam.y * Rot[4] + point_cam.z * Rot[5] + trans.y;
    z = point_cam.x * Rot[6] + point_cam.y * Rot[7] + point_cam.z * Rot[8] + trans.z;
    // -------------------------------------------------------
  }

  // Assignment operator
  point3D operator=(const point3D& other)
  {
    x = other.x;
    y = other.y;
    z = other.z;

    return *this;
  }

  // Addition operator
  point3D operator+(const point3D& other) const
  {
    point3D result(*this);
    result.x += other.x;
    result.y += other.y;
    result.z += other.z;

    return result;
  }

  // Subtraction operator
  point3D operator-(const point3D& other) const
  {
    point3D result(*this);
    result.x -= other.x;
    result.y -= other.y;
    result.z -= other.z;

    return result;
  }

  // 3D euclidean distance
  float distance(const point3D& other) const
  {
    float dist_x = x - other.x;
    float dist_y = y - other.y;
    float dist_z = z - other.z;
    return sqrt(pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_z, 2));
  }

  // 2D euclidean distance
  float distanceXY(const point3D& other) const
  {
    float dist_x = x - other.x;
    float dist_y = y - other.y;
    return sqrt(pow(dist_x, 2) + pow(dist_y, 2));
  }

  // Convert point3D to Eigen 3D vector
  Eigen::VectorXd toEig2D()
  {
    Eigen::VectorXd vec(2, 1);
    vec << x, y;
    return vec;
  }

  // Convert point3D to Eigen 3D vector
  Eigen::VectorXd toEig3D()
  {
    Eigen::VectorXd vec(3, 1);
    vec << x, y, z;
    return vec;
  }

  // Cartesian coordinates
  float x;
  float y;
  float z;

private:
};

// stdout operator
static std::ostream& operator<<(std::ostream& out, point3D const& p)
{
  return out << '(' << p.x << ' ' << p.y << ' ' << p.z << ")\n";
}
}; // namespace wildSLAM
