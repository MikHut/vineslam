#pragma once

#include "feature.hpp"

namespace vineslam
{

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

} // namespace vineslam