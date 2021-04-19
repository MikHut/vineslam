#pragma once

#include <iostream>

#include "../math/Pose.hpp"
#include "../math/Stat.hpp"
#include "../math/Point.hpp"
#include "../math/Vec.hpp"

namespace vineslam
{
struct Feature
{
  Feature() = default;

  explicit Feature(const Point& m_pos)
  {
    pos_ = m_pos;
  }

  Feature(const int& m_id, const Point& m_pos)
  {
    id_ = m_id;
    pos_ = m_pos;
  }

  bool operator==(const Feature& m_feature) const
  {
    return m_feature.pos_ == pos_;
  }

  int id_{};
  Point pos_;
};

}  // namespace vineslam
