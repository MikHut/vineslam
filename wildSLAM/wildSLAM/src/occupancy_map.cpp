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

bool OccupancyMap::insert(const Landmark& m_landmark,
                          const int&      id,
                          const int&      i,
                          const int&      j)
{
  if (i * width + j >= m_gmap.size()) {
    std::cout << "ERROR: Insert on grid map out of bounds." << std::endl;
    return false;
  } else {
    (*this)(i, j).insert(id, m_landmark);
    return true;
  }
}

bool OccupancyMap::insert(const Landmark& m_landmark,
                          const int&      id,
                          const float&    i,
                          const float&    j)
{
  // Compute grid coordinates for the floating point Landmark location
  int m_i = std::round(i / resolution);
  int m_j = std::round(j / resolution);

  return insert(m_landmark, id, m_i, m_j);
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

bool OccupancyMap::update(const Landmark& new_landmark,
                          const int&      id,
                          const float&    i,
                          const float&    j)
{
  // Compute grid coordinates for the floating point old Landmark location
  int m_i = std::round(i / resolution);
  int m_j = std::round(j / resolution);

  // Get array of landmarks present in the cell of the input landmark
  Cell                    m_cell      = (*this)(m_i, m_j);
  std::map<int, Landmark> m_landmarks = m_cell.landmarks;

  // Search for a correspondence
  for (auto m_landmark : m_landmarks) {
    if (m_landmark.first == id) {
      // Update the correspondence to the new landmark and leave the routine
      // - check if the new landmark position matches a different cell in relation
      // with previous position
      // - if so, remove the landmark from the previous cell and insert it in the
      // new correct one
      int new_m_i = std::round(new_landmark.pos.x / resolution);
      int new_m_j = std::round(new_landmark.pos.y / resolution);
      if (new_m_i != m_i || new_m_j != m_j) {
        (*this)(m_i, m_j).landmarks.erase(id);
        insert(new_landmark, id, new_landmark.pos.x, new_landmark.pos.y);
      } else {
        (*this)(m_i, m_j).landmarks[id] = new_landmark;
      }
      return true;
    }
  }
  std::cout << "WARNING: Trying to update Landmark that is not on the map... "
            << std::endl;
  return false;
}

bool OccupancyMap::getAdjacent(const int&         i,
                               const int&         j,
                               const int&         layers,
                               std::vector<Cell>& adjacent)
{
  int m_i = i - (origin.x / resolution);
  int m_j = j - (origin.y / resolution);
  if ((m_i + m_j * (width / resolution)) >= m_gmap.size()) {
    std::cout << "ERROR: Access on grid map out of bounds." << std::endl;
    return false;
  }

  if (layers == 1 || layers == 2) {
    // Compute bottom and upper bounds of indexes
    int i_min = (layers == 1) ? i - 1 : i - 2;
    int i_max = (layers == 1) ? i + 1 : i + 2;
    int j_min = (layers == 1) ? j - 1 : j - 2;
    int j_max = (layers == 1) ? j + 1 : j + 2;

    // Resize the input array
    size_t size = (layers == 1) ? 8 : 24;
    adjacent.resize(size);

    // Find and store adjacent cells
    int idx = 0;
    for (int n = i_min; n <= i_max; n++) {
      for (int m = j_min; m <= j_max; m++) {
        if (n <= (origin.x + width) / resolution && n >= origin.x / resolution &&
            m <= (origin.y + height) / resolution && m >= origin.y / resolution &&
            !(n == i && m == j)) {
          adjacent[idx] = (*this)(n, m);
          idx++;
        } else {
          adjacent[idx] = Cell();
          continue;
        }
      }
    }
    return true;
  } else {
    std::cout << "WARNING: Invalid number of adjacent layers. Only 1 or 2 adjacent "
                 "layers are supported.";
    return false;
  }
}

bool OccupancyMap::getAdjacent(const float&       i,
                               const float&       j,
                               const int&         layers,
                               std::vector<Cell>& adjacent)
{
  // Compute grid coordinates for the floating point Feature/Landmark location
  int m_i = std::round(i / resolution);
  int m_j = std::round(j / resolution);

  return getAdjacent(m_i, m_j, layers, adjacent);
}
