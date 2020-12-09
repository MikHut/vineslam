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
  }

  int id_{};                     // plane identifier
  float a_{}, b_{}, c_{}, d_{};  // plane hessian coefficients
  std::vector<Point> points_;    // set of points that belong to the plane
  std::vector<Point> indexes_;   // indexes of points projected into the range image
};

// ---------------------------------------------------------------------------------
// ----- Point cloud medium-level semi plane feature
// ---------------------------------------------------------------------------------

struct SemiPlane : public Plane
{
  SemiPlane() = default;

  SemiPlane(const float& l_a, const float& l_b, const float& l_c, const float& l_d, const std::vector<Point>& l_points,
            std::vector<Point>& l_extremas)
  {
    a_ = l_a;
    b_ = l_b;
    c_ = l_c;
    d_ = l_d;
    points_ = l_points;
    extremas_ = l_extremas;
  }

  SemiPlane(const Plane& l_plane, const std::vector<Point>& l_extremas)
  {
    a_ = l_plane.a_;
    b_ = l_plane.b_;
    c_ = l_plane.c_;
    d_ = l_plane.d_;
    points_ = l_plane.points_;
    extremas_ = l_extremas;
  }

  std::vector<Point> extremas_;
};

}  // namespace vineslam