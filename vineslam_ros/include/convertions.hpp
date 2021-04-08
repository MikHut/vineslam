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

}  // namespace vineslam