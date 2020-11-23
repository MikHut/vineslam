#pragma once

#include "feature.hpp"

namespace vineslam
{
// ---------------------------------------------------------------------------------
// ----- Image low-level feature
// ---------------------------------------------------------------------------------

struct ImageFeature : public Feature
{
  ImageFeature() = default;

  // Class constructor
  // - initializes its image/world position, color
  ImageFeature(const int& u, const int& v, const uint8_t& r, const uint8_t& g, const uint8_t& b, const Point& pos)
  {
    (*this).u_ = u;
    (*this).v_ = v;
    (*this).r_ = r;
    (*this).g_ = g;
    (*this).b_ = b;
    (*this).signature_ = std::vector<float>();
    (*this).pos_ = pos;
    (*this).n_observations_ = 0;
  }

  // Class constructor
  // - initializes its image
  ImageFeature(const int& u, const int& v)
  {
    (*this).u_ = u;
    (*this).v_ = v;
    (*this).signature_ = std::vector<float>();
    (*this).n_observations_ = 0;
  }

  int n_observations_{};
  // Image pixel position
  int u_{};
  int v_{};
  // RGB info
  uint8_t r_{};
  uint8_t g_{};
  uint8_t b_{};
  // Feature descriptor
  std::vector<float> signature_;
  // Feature laplacian - hessian matrix trace
  int laplacian_{};
};

}  // namespace vineslam