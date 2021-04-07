#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace vineslam
{
static void pose2TransformStamped(const tf2::Quaternion& q, const tf2::Vector3& t,
                                  geometry_msgs::msg::TransformStamped& tf)
{
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf.transform.translation.x = t.x();
  tf.transform.translation.y = t.y();
  tf.transform.translation.z = t.z();
}

static geometry_msgs::msg::Point point2GeometryMsgsPoint(const Point& vineslam_point)
{
  geometry_msgs::msg::Point ros_point;
  ros_point.x = vineslam_point.x_;
  ros_point.y = vineslam_point.y_;
  ros_point.z = vineslam_point.z_;

  return ros_point;
}

static std::vector<Point> voxel_traversal(Point ray_start, Point ray_end)
{
  float _bin_size = 0.25;
  float _bin_size_z = 0.25 / 8.;
  std::vector<Point> visited_voxels;

  // This id of the first/current voxel hit by the ray.
  // Using floor (round down) is actually very important,
  // the implicit int-casting will round up for negative numbers.
  Point current_voxel(std::floor(ray_start.x_ / _bin_size), std::floor(ray_start.y_ / _bin_size),
                      std::floor(ray_start.z_ / _bin_size_z));

  // The id of the last voxel hit by the ray.
  // TODO: what happens if the end point is on a border?
  Point last_voxel(std::floor(ray_end.x_ / _bin_size), std::floor(ray_end.y_ / _bin_size),
                   std::floor(ray_end.z_ / _bin_size_z));

  // Compute normalized ray direction.
  Point ray = ray_end - ray_start;
  // ray.normalize();

  // In which direction the voxel ids are incremented.
  float stepX = (ray.x_ >= 0) ? 1 : -1;  // correct
  float stepY = (ray.y_ >= 0) ? 1 : -1;  // correct
  float stepZ = (ray.z_ >= 0) ? 1 : -1;  // correct

  // Distance along the ray to the next voxel border from the current position (tMaxX, tMaxY, tMaxZ).
  float next_voxel_boundary_x = (current_voxel.x_ + stepX) * _bin_size;  // correct
  float next_voxel_boundary_y = (current_voxel.y_ + stepY) * _bin_size;  // correct
  float next_voxel_boundary_z = (current_voxel.z_ + stepZ) * _bin_size_z;  // correct

  // tMaxX, tMaxY, tMaxZ -- distance until next intersection with voxel-border
  // the value of t at which the ray crosses the first vertical voxel boundary
  float tMaxX = (ray.x_ != 0) ? (next_voxel_boundary_x - ray_start.x_) / ray.x_ : DBL_MAX;  //
  float tMaxY = (ray.y_ != 0) ? (next_voxel_boundary_y - ray_start.y_) / ray.y_ : DBL_MAX;  //
  float tMaxZ = (ray.z_ != 0) ? (next_voxel_boundary_z - ray_start.z_) / ray.z_ : DBL_MAX;  //

  // tDeltaX, tDeltaY, tDeltaZ --
  // how far along the ray we must move for the horizontal component to equal the width of a voxel
  // the direction in which we traverse the grid
  // can only be FLT_MAX if we never go in that direction
  float tDeltaX = (ray.x_ != 0) ? _bin_size / ray.x_ * stepX : DBL_MAX;
  float tDeltaY = (ray.y_ != 0) ? _bin_size / ray.y_ * stepY : DBL_MAX;
  float tDeltaZ = (ray.z_ != 0) ? _bin_size_z / ray.z_ * stepZ : DBL_MAX;

  Point diff(0, 0, 0);
  bool neg_ray = false;
  if (current_voxel.x_ != last_voxel.x_ && ray.x_ < 0)
  {
    diff.x_--;
    neg_ray = true;
  }
  if (current_voxel.y_ != last_voxel.y_ && ray.y_ < 0)
  {
    diff.y_--;
    neg_ray = true;
  }
  if (current_voxel.z_ != last_voxel.z_ && ray.z_ < 0)
  {
    diff.z_--;
    neg_ray = true;
  }
  visited_voxels.push_back(current_voxel);
  if (neg_ray)
  {
    current_voxel = current_voxel + diff;
    visited_voxels.push_back(current_voxel);
  }

  while (last_voxel != current_voxel)
  {
    if (tMaxX < tMaxY)
    {
      if (tMaxX < tMaxZ)
      {
        current_voxel.x_ += stepX;
        tMaxX += tDeltaX;
      }
      else
      {
        current_voxel.z_ += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
    else
    {
      if (tMaxY < tMaxZ)
      {
        current_voxel.y_ += stepY;
        tMaxY += tDeltaY;
      }
      else
      {
        current_voxel.z_ += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
    visited_voxels.push_back(current_voxel);
  }
  return visited_voxels;
}

}  // namespace vineslam