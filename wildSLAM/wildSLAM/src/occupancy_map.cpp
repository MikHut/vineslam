#include "occupancy_map.hpp"

OccupancyMap::OccupancyMap(const std::string& config_path)
{
  // Read input parameters
  YAML::Node config = YAML::LoadFile(config_path);
  origin.x          = config["grid_map"]["origin"]["x"].as<float>();
  origin.y          = config["grid_map"]["origin"]["y"].as<float>();
  resolution        = config["grid_map"]["resolution"].as<float>();
  width             = config["grid_map"]["width"].as<float>();
  height            = config["grid_map"]["height"].as<float>();

  // Set the grid map size
  int map_size = (width / resolution) * (height / resolution);
  m_gmap.resize(map_size);
}

bool OccupancyMap::insert(const Landmark& m_landmark, const int& i, const int& j)
{
  if (i * width + j >= m_gmap.size()) {
    std::cout << "ERROR: Insert on grid map out of bounds." << std::endl;
    return false;
  } else {
    (*this)(i, j).insert(m_landmark);
    return true;
  }
}

bool OccupancyMap::insert(const Landmark& m_landmark, const float& i, const float& j)
{
  // Compute grid coordinates for the floating point Landmark location
  int m_i = std::round(i / resolution);
  int m_j = std::round(j / resolution);

  return insert(m_landmark, m_i, m_j);
}

bool OccupancyMap::insert(const Feature& m_feature, const int& i, const int& j)
{
  if (i * width + j >= m_gmap.size()) {
    std::cout << "ERROR: Insert on grid map out of bounds." << std::endl;
    return false;
  } else {
    (*this)(i, j).features.push_back(m_feature);
    return true;
  }
}

bool OccupancyMap::insert(const Feature& m_feature, const float& i, const float& j)
{
  // Compute grid coordinates for the floating point Feature location
  int m_i = std::round(i / resolution);
  int m_j = std::round(j / resolution);

  return insert(m_feature, m_i, m_j);
}

bool OccupancyMap::update(const Landmark& old_landmark,
                          const Landmark& new_landmark,
                          const float&    i,
                          const float&    j)
{
  // Compute grid coordinates for the floating point old Landmark location
  int m_i = std::round(i / resolution);
  int m_j = std::round(j / resolution);

  // Get array of landmarks present in the cell of the input landmark
  Cell                  m_cell      = (*this)(m_i, m_j);
  std::vector<Landmark> m_landmarks = m_cell.landmarks;

  // Search for a correspondence
  for (size_t k = 0; k < m_landmarks.size(); k++) {
    if (m_landmarks[k].pos.x == old_landmark.pos.x &&
        m_landmarks[k].pos.y == old_landmark.pos.y) {
      // Update the correspondence to the new landmark and leave the routine
      // - check if the new landmark position matches a different cell in relation
      // with previous position
      // - if so, remove the landmark from the previous cell and insert it in the
      // new correct one
      int new_m_i = std::round(new_landmark.pos.x / resolution);
      int new_m_j = std::round(new_landmark.pos.y / resolution);
      if (new_m_i != m_i || new_m_j != m_j) {
        (*this)(m_i, m_j).landmarks.erase((*this)(m_i, m_j).landmarks.begin() + k);
        insert(new_landmark, new_landmark.pos.x, new_landmark.pos.y);
      } else
        (*this)(m_i, m_j).landmarks[k] = new_landmark;
      return true;
    }
  }

  std::cout << "WARNING: Trying to update Landmark that is not on the map... "
            << std::endl;
  return false;
}

bool OccupancyMap::getAdjacent(const Cell&          m_cell,
                               const int&           i,
                               const int&           j,
                               std::array<Cell, 8>& adjacent)
{
  if (i * width + j >= m_gmap.size()) {
    std::cout << "ERROR: Access on grid map out of bounds." << std::endl;
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

bool OccupancyMap::getAdjacent(const Cell&          m_cell,
                               const float&         i,
                               const float&         j,
                               std::array<Cell, 8>& adjacent)
{
  // Compute grid coordinates for the floating point Feature/Landmark location
  int m_i = std::round(i / resolution);
  int m_j = std::round(j / resolution);

  return getAdjacent(m_cell, m_i, m_j, adjacent);
}
