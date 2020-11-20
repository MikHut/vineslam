#pragma once

#include <iostream>
#include <array>
#include <cmath>

namespace vineslam
{

const float DEGREE_TO_RAD = static_cast<float>(M_PI / 180.);
const float RAD_TO_DEGREE = static_cast<float>(180. / M_PI);
const float M_2PI         = static_cast<float>(2. * M_PI);

// Normalize an angle between -PI and PI
static float normalizeAngle(const float& angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace vineslam