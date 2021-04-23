#pragma once

#include <vineslam/params.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Pose.hpp>

#include <iostream>
#include <vector>
#include <cmath>

namespace vineslam
{
class ElevationMap
{
public:
  // Default constructor
  ElevationMap() = default;

  // Copy constructor
  ElevationMap(const ElevationMap& elevation_map);

  // Class constructor
  // - initializes the elevation map given the input parameters
  ElevationMap(const Parameters& params, const Pose& origin_offset);

  // 2D grid map direct access to cell coordinates
  float& operator()(int i, int j)
  {
    // Verify validity of indexing
    try
    {
      check(i, j);
    }
    catch (char const* msg)
    {
#if VERBOSE == 1
      std::cout << msg;
      std::cout << "Returning last grid element ..." << std::endl;
#endif

      return cell_vec_[cell_vec_.size() - 1];
    }

    // Compute indexes having into account that grid map considers negative values
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
    int l_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
    int idx = l_i + l_j * static_cast<int>(std::round(width_ / resolution_ + .49));

    return cell_vec_[idx];
  }

  // 2D grid map access given a Feature/Landmark location
  float& operator()(float i, float j)
  {
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = static_cast<int>(std::round(i / resolution_ + .49));
    int l_j = static_cast<int>(std::round(j / resolution_ + .49));

    return (*this)(l_i, l_j);
  }

  // Define iterator to provide access to the cells array
  typedef std::vector<float>::iterator iterator;
  // Return members to provide access to the cells array
  iterator begin()
  {
    return cell_vec_.begin();
  }
  iterator end()
  {
    return cell_vec_.end();
  }

  // Update the cell altimetry using direct grid coordinates
  bool update(const float& z, const int& i, const int& j);
  // Update the cell altimetry using the (x,y) location
  bool update(const float& z, const float& i, const float& j);

  // Get elevation color
  static void color(float z, float& r, float& g, float& b);

  // Grid map dimensions
  Point origin_;
  float resolution_{};
  float width_{};
  float lenght_{};

private:
  // Private grid map to store all the cells
  std::vector<float> cell_vec_;

  // Check out of bounds indexing
  void check(const int& i, const int& j)
  {
    // Compute indexes having into account that grid map considers negative values
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
    int l_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
    int index = l_i + (l_j * static_cast<int>(std::round(width_ / resolution_ + .49)));

    // Trough exception if out of bounds indexing
    if (index >= static_cast<int>(cell_vec_.size()) - 1 || index < 0)
      throw "Exception: Access to grid map out of bounds\n";
  }
};
}  // namespace vineslam
