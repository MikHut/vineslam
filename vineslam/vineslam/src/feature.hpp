#pragma once

#include <iostream>
#include <math/pose.hpp>
#include <math/stat.hpp>
#include <math/point.hpp>
#include <math/vector3D.hpp>

namespace vineslam
{

struct Feature {
  Feature() = default;

  explicit Feature(const point& m_pos) { pos = m_pos; }

  Feature(const int& m_id, const point& m_pos)
  {
    id  = m_id;
    pos = m_pos;
  }

  bool operator==(Feature m_feature)
  {
    return m_feature.pos == pos;
  }

  int   id{};
  point pos;
};

// ---------------------------------------------------------------------------------
// ----- Semantic high-level feature
// ---------------------------------------------------------------------------------

// Structure to represent the semantic information about
// each feature:
// - type of feature
// - description of the feature
// - dynamic or static
struct SemanticInfo {
  SemanticInfo() = default;
  SemanticInfo(const std::string& type,
               const std::string& description,
               const int&         character)
  {
    (*this).type        = type;
    (*this).description = description;
    (*this).character   = character;
  }

  // Initialize semantic information of feature to give
  // to the mapper class
  explicit SemanticInfo(const int& label)
  {
    std::string m_type;
    std::string m_desc;
    int         m_ch;

    switch (label) {
      case 0:
        m_type = "Trunk";
        m_desc = "Vine trunk. A static landmark";
        m_ch   = 0;

        *this = SemanticInfo(m_type, m_desc, m_ch);
        break;
      case 1:
        type   = "Leaf";
        m_desc = "Leaf from a vine trunk. A dynamic landmark";
        m_ch   = 1;

        *this = SemanticInfo(type, m_desc, m_ch);
        break;
      default:
        *this = SemanticInfo("Trunk", "Vine trunk", 0);
    }
  }

  std::string type;
  std::string description;
  int         character{};
};

struct SemanticFeature : public Feature {
  SemanticFeature() = default;
  // Class constructor
  // - initializes its pose, standard deviation and
  // - its sematic information
  SemanticFeature(const point&                  pos,
                  const Gaussian<point, point>& gauss,
                  const int&                    label)
  {
    (*this).pos   = pos;
    (*this).gauss = gauss;
    (*this).info  = SemanticInfo(label);
  }
  // Class constructor
  // - initializes its pose, standard deviation
  SemanticFeature(const point& pos, const Gaussian<point, point>& gauss)
  {
    (*this).pos   = pos;
    (*this).gauss = gauss;
  }

  // Print semantic landmark information
  void print()
  {
    std::string c = (info.character == 0) ? "static" : "dynamic";

    std::cout << "Landmark " << std::endl;
    std::cout << "   type:        " << info.type << std::endl;
    std::cout << "   description: " << info.description << std::endl;
    std::cout << "   character:   " << c << std::endl;
    std::cout << "   position:    " << pos;
    std::cout << "   stdev:      [" << gauss.stdev.x << "," << gauss.stdev.y << "]"
              << std::endl;
  }

  Gaussian<point, point> gauss;
  SemanticInfo           info;
};

// ---------------------------------------------------------------------------------
// ----- Image low-level feature
// ---------------------------------------------------------------------------------

struct ImageFeature : public Feature {
  ImageFeature() = default;

  // Class constructor
  // - initializes its image/world position, color
  ImageFeature(const int&     u,
               const int&     v,
               const uint8_t& r,
               const uint8_t& g,
               const uint8_t& b,
               const point&   pos)
  {
    (*this).u              = u;
    (*this).v              = v;
    (*this).r              = r;
    (*this).g              = g;
    (*this).b              = b;
    (*this).signature      = std::vector<float>();
    (*this).pos            = pos;
    (*this).n_observations = 0;
  }

  // Class constructor
  // - initializes its image
  ImageFeature(const int& u, const int& v)
  {
    (*this).u              = u;
    (*this).v              = v;
    (*this).signature      = std::vector<float>();
    (*this).n_observations = 0;
  }

  int n_observations{};
  // Image pixel position
  int u{};
  int v{};
  // RGB info
  uint8_t r{};
  uint8_t g{};
  uint8_t b{};
  // Feature descriptor
  std::vector<float> signature;
  // Feature laplacian - hessian matrix trace
  int laplacian{};
};

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
  int which_cluster{}; // sets the cluster where the corner belongs
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
  int which_cluster{}; // sets the cluster where the corner belongs
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
// ----- Point cloud medium-level sphere feature
// ---------------------------------------------------------------------------------
struct Cluster : public Feature {
  Cluster() = default;

  Cluster(const point& m_center, const point& m_radius, const int& m_id = 0)
  {
    center = m_center;
    radius = m_radius;
    id     = m_id;
  }

  Cluster(const point&               m_center,
          const point&               m_radius,
          const std::vector<Corner>& m_items,
          const int&                 m_id = 0)
  {
    center = m_center;
    radius = m_radius;
    id     = m_id;

    items = m_items;
  }

  point center; // Cluster centre
  point radius; // Cluster radius

  std::vector<Corner> items; // Items located inside the cluster
};

// ---------------------------------------------------------------------------------
// ----- Point cloud medium-level line feature
// ---------------------------------------------------------------------------------

struct Line {
  Line() = default;

  Line(const float& m_m, const float& m_b)
  {
    m = m_m;
    b = m_b;
  }

  // - This constructor fits a line in a set of points using a linear regression
  explicit Line(const std::vector<point>& m_pts)
  {
    float sumX = 0., sumX2, sumY = 0., sumXY = 0.;
    float n      = 0;
    float x_mean = 0., x_max = 0.;
    for (const auto& pt : m_pts) {
      sumX += pt.x;
      sumX2 += pt.x * pt.x;
      sumY += pt.y;
      sumXY += pt.x * pt.y;

      x_mean += pt.x;
      x_max = (pt.x > x_max) ? pt.x : x_max;
      n += 1.;
    }

    if (n == 0) {
      m = 0;
      b = 0;
    } else {
      m = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
      b = (sumY - m * sumX) / n;
    }
  }

  float dist(const point& pt)
  {
    return std::sqrt(std::pow(b + m * pt.x - pt.y, 2) / std::pow(m * m + 1, 2));
  }

  float m; // slope
  float b; // zero intercept
};

// ---------------------------------------------------------------------------------
// ----- Point cloud medium-level plane feature
// ---------------------------------------------------------------------------------

struct Plane {
  Plane() = default;

  explicit Plane(const vector3D& m_normal) { normal = m_normal; }

  Plane(const vector3D& m_normal, const std::vector<point>& m_points)
  {
    normal = m_normal;
    points = m_points;
  }

  int                id{};    // plane identifier
  vector3D           normal;  // plane normal vector
  std::vector<point> points;  // set of points that belong to the plane
  std::vector<point> indexes; // indexes of points projected into the range image
  Line               regression{};  // XY linear fitting of plane points into a line
  float              mean_height{}; // ground plane mean height - used for validation
};

} // namespace vineslam
