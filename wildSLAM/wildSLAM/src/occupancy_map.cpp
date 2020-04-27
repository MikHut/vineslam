#include "occupancy_map.hpp"

OccupancyMap::OccupancyMap(const std::string& config_path)
{
  // Read input parameters
  YAML::Node config     = YAML::LoadFile(config_path.c_str());
  int        dimX       = config["grid_map"]["dimension_x"].as<int>();
  int        dimY       = config["grid_map"]["dimension_y"].as<int>();
  float      resolution = config["grid_map"]["resolution"].as<float>();

  // Compute grid map size
  width  = dimX / resolution;
  height = dimY / resolution;

  std::cout << width << "," << height << std::endl;

  // Set the grid map size
  m_gmap.resize(width * height);
}

bool OccupancyMap::insert(const Cell& m_cell, const int& i, const int& j)
{
  if (i * width + j >= m_gmap.size()) {
    std::cout << "ERROR: Insert on grid map out of bounds." << std::endl;
    return false;
  } else {
    (*this)(i, j) = m_cell;
    return true;
  }
}
bool OccupancyMap::getAdjacent(const Cell&          m_cell,
                               const int&           i,
                               const int&           j,
                               std::array<Cell, 8>& adjacent)
{
  if (i * width + j >= m_gmap.size()) {
    std::cout << "ERROR: Insert on grid map out of bounds." << std::endl;
    return false;
  } else {
    // Upper cell
    int i_upper = i + 1;
    int j_upper = j;
    // Bottom cell
    int i_bottom = i - 1;
    int j_bottom = j;
    // Right cell
    int i_right = i;
    int j_right = j + 1;
    // Left cell
    int i_left = i;
    int j_left = j - 1;
    // Upper right cell
    int i_upperright = i + 1;
    int j_upperright = j + 1;
    // Upper left cell
    int i_upperleft = i + 1;
    int j_upperleft = j - 1;
    // Bottom right cell
    int i_bottomright = i - 1;
    int j_bottomright = j + 1;
    // Bottom left cell
    int i_bottomleft = i - 1;
    int j_bottomleft = j - 1;

    // Fill the adjacent cells array
    adjacent[0] = (*this)(i_upper, j_upper);
    adjacent[1] = (*this)(i_bottom, j_bottom);
    adjacent[2] = (*this)(i_right, j_right);
    adjacent[3] = (*this)(i_left, j_left);
    adjacent[4] = (*this)(i_upperright, j_upperright);
    adjacent[5] = (*this)(i_upperleft, j_upperleft);
    adjacent[6] = (*this)(i_bottomright, j_bottomright);
    adjacent[7] = (*this)(i_bottomleft, j_bottomleft);

    return true;
  }
}
