#include <vineslam/mapping/elevation_map.hpp>

namespace vineslam
{
ElevationMap::ElevationMap(const Parameters& params, const Pose& origin_offset)
{
  // Read input parameters
  origin_.x_ = params.gridmap_origin_x_ + origin_offset.x_;
  origin_.y_ = params.gridmap_origin_y_ + origin_offset.y_;
  resolution_ = params.gridmap_resolution_;
  width_ = params.gridmap_width_;
  lenght_ = params.gridmap_lenght_;

  // Set the grid map size
  int map_size = static_cast<int>(std::round((width_ / resolution_) * (lenght_ / resolution_)));
  cell_vec_ = std::vector<float>(map_size, 0);
}

ElevationMap::ElevationMap(const ElevationMap& elevation_map)
{
  this->cell_vec_ = elevation_map.cell_vec_;
  this->resolution_ = elevation_map.resolution_;
  this->origin_ = elevation_map.origin_;
  this->lenght_ = elevation_map.lenght_;
  this->width_ = elevation_map.width_;
}

bool ElevationMap::update(const float& z, const int& i, const int& j)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
#if VERBOSE == 1
    std::cout << "ElevationMap::update() --- " << msg;
#endif
    return false;
  }

  (*this)(i, j)  = (z > (*this)(i, j) || (*this)(i,j) == 0) ? z : (*this)(i, j);

  return true;
}

bool ElevationMap::update(const float& z, const float& i, const float& j)
{
  // Compute grid coordinates for the floating point Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int l_i = static_cast<int>(std::round(i / resolution_ + .49));
  int l_j = static_cast<int>(std::round(j / resolution_ + .49));

  return update(z, l_i, l_j);
}

void ElevationMap::color(float z, float& r, float&g, float& b)
{
  // blend over HSV-values (more colors)
  float s = 1.0;
  float v = 1.0;

  z -= std::floor(z);
  z *= 6;
  int i;
  float m, n, f;

  i = std::floor(z);
  f = z - static_cast<float>(i);
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      r = v; g = n; b = m;
      break;
    case 1:
      r = n; g = v; b = m;
      break;
    case 2:
      r = m; g = v; b = n;
      break;
    case 3:
      r = m; g = n; b = v;
      break;
    case 4:
      r = n; g = m; b = v;
      break;
    case 5:
      r = v; g = m; b = n;
      break;
    default:
      r = 1; g = 0.5; b = 0.5;
      break;
  }
}

}  // namespace vineslam