#pragma once

#include <iostream>

#include "../math/pose.hpp"
#include "../math/stat.hpp"
#include "../math/point.hpp"
#include "../math/vector3D.hpp"

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

  bool operator==(Feature m_feature) { return m_feature.pos == pos; }

  int   id{};
  point pos;
};

} // namespace vineslam
