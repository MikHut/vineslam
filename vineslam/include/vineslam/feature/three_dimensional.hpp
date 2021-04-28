#pragma once

#include "feature.hpp"

namespace vineslam
{
// ---------------------------------------------------------------------------------
// ----- Point cloud medium-level corner feature
// ---------------------------------------------------------------------------------

struct Corner : public Feature
{
  Corner() = default;

  Corner(const Point& l_pt, const int& l_which_plane, const int& l_id = 0)
  {
    pos_ = l_pt;
    which_plane_ = l_which_plane;
    id_ = l_id;
    n_observations_ = 0;
  }

  int n_observations_{};
  int which_plane_{};  // sets the plane where the corner belongs
};

// ---------------------------------------------------------------------------------
// ----- Point cloud medium-level planar feature
// ---------------------------------------------------------------------------------

struct Planar : public Feature
{
  Planar() = default;

  Planar(const Point& l_pt, const int& l_which_plane, const int& l_id = 0)
  {
    pos_ = l_pt;
    which_plane_ = l_which_plane;
    id_ = l_id;
    n_observations_ = 0;
  }

  int n_observations_{};
  int which_plane_{};  // sets the plane where the corner belongs
};

// Dummy struct to represent a plane point, before corner extraction
struct PlanePoint : public Corner
{
  PlanePoint() = default;

  PlanePoint(const Point& l_pt, const int& l_which_plane)
  {
    pos_ = l_pt;
    which_plane_ = l_which_plane;
  }

  explicit PlanePoint(const Corner& l_corner)
  {
    pos_ = l_corner.pos_;
    which_plane_ = l_corner.which_plane_;
  }
};

// ---------------------------------------------------------------------------------
// ----- Point cloud medium-level plane feature
// ---------------------------------------------------------------------------------

struct Plane
{
  Plane() = default;

  Plane(const float& l_a, const float& l_b, const float& l_c, const float& l_d, const std::vector<Point>& l_points)
  {
    a_ = l_a;
    b_ = l_b;
    c_ = l_c;
    d_ = l_d;
    points_ = l_points;
    setLocalRefFrame();
  }

  void setLocalRefFrame()
  {
    // Define a point in the plane which is the first of the list, projected to the plane
    // use average y and z values (since velodyne uses x forward) to put origin on the center of the pattern
    Point p0(0, 0, 0);
    for (const auto& pt : points_)
    {
      p0.y_ += pt.y_;
      p0.z_ += pt.z_;
    }
    p0.y_ /= static_cast<float>(points_.size());
    p0.z_ /= static_cast<float>(points_.size());
    p0.x_ = (-b_ * p0.y_ - c_ * p0.z_ - d_) / a_;

    // Define a second point. We use the first point from the list of points, and project it to the plane
    Point p1(0, 0, 0);
    p1.y_ = points_[0].y_;
    p1.z_ = points_[0].z_;
    p1.x_ = (-b_ * p1.y_ - c_ * p1.z_ - d_) / a_;

    // Define a rotation matrix, and the n (direction vector along the x axis), s and a vectors:
    //        n    s    a
    // R = [ r11  r12  r13 ]
    //     [ r21  r22  r23 ]
    //     [ r31  r32  r33 ]

    // Semi-arbitrary x-axis: only constraint is that it is coplanar with the plane. This is
    // ensured since both p0 and p1 lie on the plane
    Vec n(p1, p0);
    Vec a(a_, b_, c_);   // z axis must have the direction normal to the plane
    Vec s = a.cross(n);  // y axis is the cross product of z axis per the x axis

    // Normalize the three vectors
    n.normalize();
    a.normalize();
    s.normalize();

    // Compute final transformation matrix
    local_ref_.R_array_ = { n.x_, s.x_, a.x_, n.y_, s.y_, a.y_, n.z_, s.z_, a.z_ };
    local_ref_.t_array_ = { p0.x_, p0.y_, p0.z_ };  // We use one the points in the plane to define the translation
  }

  float point2Plane(const Point& point) const
  {
    auto norm = std::sqrt(a_ * a_ + b_ * b_ + c_ * c_);
    return std::fabs(a_ * point.x_ + b_ * point.y_ + c_ * point.z_ + d_) / norm;
  }

  int id_{};                     // plane identifier
  float a_{}, b_{}, c_{}, d_{};  // plane hessian coefficients
  std::vector<Point> points_;    // set of points that belong to the plane
  std::vector<Point> indexes_;   // indexes of points projected into the range image
  Point centroid_{};             // plane centroid
  Tf local_ref_;                 // local reference frame
};

// ---------------------------------------------------------------------------------
// ----- Point cloud medium-level semi plane feature
// ---------------------------------------------------------------------------------

struct SemiPlane : public Plane
{
  SemiPlane() = default;

  SemiPlane(const Plane& l_plane, const std::vector<Point>& l_extremas)
  {
    a_ = l_plane.a_;
    b_ = l_plane.b_;
    c_ = l_plane.c_;
    d_ = l_plane.d_;
    points_ = l_plane.points_;
    centroid_ = l_plane.centroid_;
    local_ref_ = l_plane.local_ref_;
    extremas_ = l_extremas;
    n_correspondences_ = 0;
    n_occurences_ = 0;
    transformAndSetArea();
  }

  // Computes the semi-plane area
  // Shoelace formula: https://en.wikipedia.org/wiki/Shoelace_formula
  void setArea()
  {
    float a = 0;

    for (size_t i = 1; i < extremas_.size(); i++)
    {
      Point pe = extremas_[i - 1];
      Point ce = extremas_[i];

      a += (pe.x_ * ce.y_ - pe.y_ * ce.x_);
    }

    area_ = std::fabs(a / 2);
  }

  // Transforms the semi-plane to its local reference frame and then computes the semi-plane area
  // Shoelace formula: https://en.wikipedia.org/wiki/Shoelace_formula
  void transformAndSetArea()
  {
    std::vector<Point> extremas;
    for (const auto& extrema : extremas_)
    {
      Point p = extrema * local_ref_;
      p.z_ = 0;
      extremas.push_back(p);
    }

    float a = 0;

    for (size_t i = 1; i < extremas.size(); i++)
    {
      Point pe = extremas[i - 1];
      Point ce = extremas[i];

      a += (pe.x_ * ce.y_ - pe.y_ * ce.x_);
    }

    area_ = std::fabs(a / 2);
  }

  uint32_t n_occurences_;
  uint32_t n_correspondences_;
  std::vector<Point> extremas_;
  float area_{};
};

}  // namespace vineslam