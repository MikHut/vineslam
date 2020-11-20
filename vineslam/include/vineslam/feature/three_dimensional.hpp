#pragma once

#include "feature.hpp"

namespace vineslam
{

// ---------------------------------------------------------------------------------
// ----- Point cloud medium-level corner feature
// ---------------------------------------------------------------------------------

struct Corner : public Feature {
  Corner() = default;

  Corner(const point& m_pt, const int& m_which_plane, const int& m_id = 0)
  {
    pos            = m_pt;
    which_plane    = m_which_plane;
    id             = m_id;
    n_observations = 0;
  }

  int n_observations{};
  int which_plane{};   // sets the plane where the corner belongs
};

// ---------------------------------------------------------------------------------
// ----- Point cloud medium-level planar feature
// ---------------------------------------------------------------------------------

struct Planar : public Feature {
  Planar() = default;

  Planar(const point& m_pt, const int& m_which_plane, const int& m_id = 0)
  {
    pos            = m_pt;
    which_plane    = m_which_plane;
    id             = m_id;
    n_observations = 0;
  }

  int n_observations{};
  int which_plane{};   // sets the plane where the corner belongs
};

// Dummy struct to represent a plane point, before corner extraction
struct PlanePoint : public Corner {
  PlanePoint() = default;

  PlanePoint(const point& m_pt, const int& m_which_plane)
  {
    pos         = m_pt;
    which_plane = m_which_plane;
  }

  explicit PlanePoint(const Corner& m_corner)
  {
    pos         = m_corner.pos;
    which_plane = m_corner.which_plane;
  }
};

// ---------------------------------------------------------------------------------
// ----- Point cloud medium-level plane feature
// ---------------------------------------------------------------------------------

struct Plane {
  Plane() = default;

  Plane(const float&              m_a,
        const float&              m_b,
        const float&              m_c,
        const float&              m_d,
        const std::vector<point>& m_points)
  {
    a      = m_a;
    b      = m_b;
    c      = m_c;
    d      = m_d;
    points = m_points;
  }

  // RANSAC routine
  bool ransac(const Plane& in_plane, int max_iters = 20, float dist_threshold = 0.08)
  {
    int max_idx       = in_plane.points.size();
    int min_idx       = 0;
    int max_tries     = 1000;
    int c_max_inliers = 0;

    for (int i = 0; i < max_iters; i++) {
      // Declare private point cloud to store current solution
      std::vector<point> m_pcl;

      // Reset number of inliers in each iteration
      int num_inliers = 0;

      // Randomly select three points that cannot be cohincident
      // TODO (André Aguiar): Also check if points are collinear
      bool found_valid_pts = false;
      int  n               = 0;
      int  idx1, idx2, idx3;
      while (!found_valid_pts) {
        idx1 = std::rand() % (max_idx - min_idx + 1) + min_idx;
        idx2 = std::rand() % (max_idx - min_idx + 1) + min_idx;
        idx3 = std::rand() % (max_idx - min_idx + 1) + min_idx;

        if (idx1 != idx2 && idx1 != idx3 && idx2 != idx3)
          found_valid_pts = true;

        n++;
        if (n > max_tries)
          break;
      }

      if (!found_valid_pts) {
        std::cout << "WARNING (ransac): No valid set of points found ... "
                  << std::endl;
        return false;
      }

      // Declarate the 3 points selected on this iteration
      point pt1 = point(
          in_plane.points[idx1].x, in_plane.points[idx1].y, in_plane.points[idx1].z);
      point pt2 = point(
          in_plane.points[idx2].x, in_plane.points[idx2].y, in_plane.points[idx2].z);
      point pt3 = point(
          in_plane.points[idx3].x, in_plane.points[idx3].y, in_plane.points[idx3].z);

      // Extract the plane hessian coefficients
      vector3D v1(pt2, pt1);
      vector3D v2(pt3, pt1);
      vector3D abc = v1.cross(v2);
      float    m_a = abc.x;
      float    m_b = abc.y;
      float    m_c = abc.z;
      float    m_d = -(m_a * pt1.x + m_b * pt1.y + m_c * pt1.z);

      for (const auto& m_pt : in_plane.points) {
        // Compute the distance each point to the plane - from
        // https://www.geeksforgeeks.org/distance-between-a-point-and-a-plane-in-3-d/
        auto norm = std::sqrt(m_a * m_a + m_b * m_b + m_c * m_c);
        if (std::fabs(m_a * m_pt.x + m_b * m_pt.y + m_c * m_pt.z + m_d) / norm <
            dist_threshold) {
          num_inliers++;
          m_pcl.push_back(m_pt);
        }
      }

      if (num_inliers > c_max_inliers) {
        c_max_inliers = num_inliers;

        points.clear();
        points = m_pcl;
        a      = m_a;
        b      = m_b;
        c      = m_c;
        d      = m_d;
      }
    }

    return c_max_inliers > 0;
  }

  int                id{};               // plane identifier
  float              a{}, b{}, c{}, d{}; // plane hessian coefficients
  std::vector<point> points;             // set of points that belong to the plane
  std::vector<point> indexes; // indexes of points projected into the range image
};

} // namespace vineslam