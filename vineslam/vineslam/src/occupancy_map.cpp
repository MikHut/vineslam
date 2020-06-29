#include "occupancy_map.hpp"

namespace vineslam
{

OccupancyMap::OccupancyMap(const std::string& config_path)
{
  // Read input parameters
  YAML::Node config = YAML::LoadFile(config_path);
  origin.x          = config["grid_map"]["origin"]["x"].as<float>();
  origin.y          = config["grid_map"]["origin"]["y"].as<float>();
  resolution        = config["grid_map"]["resolution"].as<float>();
  width             = config["grid_map"]["width"].as<float>();
  height            = config["grid_map"]["height"].as<float>();
  metric            = config["grid_map"]["metric"].as<std::string>();

  // Set the grid map size
  int map_size =
      static_cast<int>(std::round((width / resolution) * (height / resolution)));
  m_gmap.resize(map_size);

  // Initialize number of features and landmarks
  n_surf_features = 0;
  n_landmarks     = 0;
}

OccupancyMap::OccupancyMap(const OccupancyMap& grid_map)
{
  this->m_gmap          = grid_map.m_gmap;
  this->n_surf_features = grid_map.n_surf_features;
  this->n_landmarks     = grid_map.n_landmarks;
  this->resolution      = grid_map.resolution;
  this->origin          = grid_map.origin;
  this->height          = grid_map.height;
  this->width           = grid_map.width;
  this->metric          = grid_map.metric;
}

bool OccupancyMap::insert(const SemanticFeature& m_landmark,
                          const int&             id,
                          const int&             i,
                          const int&             j)
{
  try {
    check(i, j);
  } catch (char const* msg) {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).insert(id, m_landmark);
  n_landmarks++;
  return true;
}

bool OccupancyMap::insert(const SemanticFeature& m_landmark, const int& id)
{
  // Compute grid coordinates for the floating point Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_landmark.pos.x / resolution + .49));
  int m_j = static_cast<int>(std::round(m_landmark.pos.y / resolution + .49));

  return insert(m_landmark, id, m_i, m_j);
}

bool OccupancyMap::insert(const ImageFeature& m_feature, const int& i, const int& j)
{
  try {
    check(i, j);
  } catch (char const* msg) {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).surf_features.push_back(m_feature);
  n_surf_features++;
  return true;
}

bool OccupancyMap::insert(const ImageFeature& m_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_feature.pos.x / resolution + .49));
  int m_j = static_cast<int>(std::round(m_feature.pos.y / resolution + .49));

  return insert(m_feature, m_i, m_j);
}

bool OccupancyMap::insert(const Corner& m_feature, const int& i, const int& j)
{
  try {
    check(i, j);
  } catch (char const* msg) {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).corner_features.push_back(m_feature);
  n_corner_features++;
  return true;
}

bool OccupancyMap::insert(const Corner& m_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_feature.pos.x / resolution + .49));
  int m_j = static_cast<int>(std::round(m_feature.pos.y / resolution + .49));

  return insert(m_feature, m_i, m_j);
}

bool OccupancyMap::update(const SemanticFeature& new_landmark,
                          const int&             id,
                          const float&           i,
                          const float&           j)
{
  // Compute grid coordinates for the floating point old Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(i / resolution + .49));
  int m_j = static_cast<int>(std::round(j / resolution + .49));

  // Get array of landmarks present in the cell of the input landmark
  Cell                           m_cell      = (*this)(m_i, m_j);
  std::map<int, SemanticFeature> m_landmarks = m_cell.landmarks;

  // Search for a correspondence
  for (const auto& m_landmark : m_landmarks) {
    if (m_landmark.first == id) {
      // Update the correspondence to the new landmark and leave the routine
      // - check if the new landmark position matches a different cell in relation
      // with previous position
      // - if so, remove the landmark from the previous cell and insert it in the
      // new correct one
      // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
      int new_m_i =
          static_cast<int>(std::round(new_landmark.pos.x / resolution + .49));
      int new_m_j =
          static_cast<int>(std::round(new_landmark.pos.y / resolution + .49));
      if (new_m_i != m_i || new_m_j != m_j) {
        (*this)(m_i, m_j).landmarks.erase(id);
        insert(new_landmark, id);
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
  try {
    check(i, j);
  } catch (char const* msg) {
    std::cout << msg;
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
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(i / resolution + .49));
  int m_j = static_cast<int>(std::round(j / resolution + .49));

  return getAdjacent(m_i, m_j, layers, adjacent);
}

bool OccupancyMap::findNearest(const ImageFeature& input,
                               ImageFeature&       nearest,
                               float&              min_dist)
{
  if (n_surf_features == 0) {
    std::cout
        << "WARNING (findNearest): Trying to find nearest feature on empty map..."
        << std::endl;
    return false;
  }

  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int i = static_cast<int>(std::round(input.pos.x / resolution + .49));
  int j = static_cast<int>(std::round(input.pos.y / resolution + .49));

  // Enumerator used to go through the nearest neighbor search
  enum moves { ORIGIN, RIGHT, DOWN, LEFT, UP, DONE };
  moves move = ORIGIN;

  // Target cell current index
  int m_i, m_j;
  // level of search (level = 1 means searching on adjacent cells)
  int level = 0;
  // iterator to move into the desired next cell
  int it = 0;
  // distance checker
  min_dist = 1e6;
  // booleans for stop criteria
  bool found_solution;
  bool valid_iteration;

  do {
    valid_iteration = false;
    found_solution  = false;
    do {
      switch (move) {
        case ORIGIN:
          try {
            check(i, j);
          } catch (char const* msg) {
            std::cout << msg;
            return false;
          }

          // Set cell indexes where to find correspondences
          m_i = i;
          m_j = j;
          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // End search if we found a correspondence in the source cell
          move = DONE;
          break;
        case RIGHT:
          // Compute cell indexes
          m_i = i - level + it;
          m_j = j + level;
          try {
            check(m_i, m_j);
          } catch (char const* msg) {
            move = DOWN;
            it   = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Found solution if there is any feature in the target cell
          if (m_i == i + level) {
            move = DOWN;
            it   = 1;
          } else {
            it++;
          }
          break;
        case DOWN:
          // Compute cell indexes
          m_i = i + level;
          m_j = j + level - it;
          try {
            check(m_i, m_j);
          } catch (char const* msg) {
            move = LEFT;
            it   = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (m_j == j - level) {
            move = LEFT;
            it   = 1;
          } else {
            it++;
          }
          break;
        case LEFT:
          // Compute cell indexes
          m_i = i + level - it;
          m_j = j - level;
          try {
            check(m_i, m_j);
          } catch (char const* msg) {
            move = UP;
            it   = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (m_i == i - level) {
            move = UP;
            it   = 1;
          } else {
            it++;
          }
          break;
        case UP:
          // Compute cell indexes
          m_i = i - level;
          m_j = j - level + it;
          try {
            check(m_i, m_j);
          } catch (char const* msg) {
            it   = 0;
            move = DONE;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          // The '-1' is to not repeat the first iterator (started on RIGHT)
          if (m_j == j + level - 1) {
            move = DONE;
            it   = 0;
          } else {
            it++;
          }
          break;
        case DONE:
          move = RIGHT;
          it   = 0;
          continue;
      }

      // ---------------------------------------------------------------------------
      if (metric == "euclidean") {
        // Found solution if there is any feature in the target cell
        found_solution = found_solution | !(*this)(m_i, m_j).surf_features.empty();

        // ------- Use euclidean distance to find correspondences
        // ------- Grid map is used to limit the search space
        for (const auto& feature : (*this)(m_i, m_j).surf_features) {
          float dist = input.pos.distance(feature.pos);
          if (dist < min_dist) {
            min_dist = dist;
            nearest  = feature;
          }
        }
      } else {
        // ------- Use feature descriptor to find correspondences
        // ------- Grid map is used to limit the search space
        for (const auto& feature : (*this)(m_i, m_j).surf_features) {
          std::vector<float> desc   = input.signature;
          std::vector<float> m_desc = feature.signature;

          // Check validity of descriptors data
          if (desc.size() != m_desc.size()) {
            std::cout << "WARNING (findNearest): source and target descriptor have "
                         "different size ... "
                      << std::endl;
            break;
          }

          // Check if source and target features are of the same type
          if (feature.laplacian != input.laplacian)
            continue;

          // Found solution if there is any correspondence between features of the
          // same type
          found_solution = true;

          float ssd = 0.; // sum of square errors
          for (size_t k = 0; k < desc.size(); k++)
            ssd += (desc[k] - m_desc[k]) * (desc[k] - m_desc[k]);

          // Update correspondence if we found a local minimum
          if (ssd < min_dist) {
            min_dist = ssd;
            nearest  = feature;
          }
        }
      }
      // ---------------------------------------------------------------------------

    } while (move != DONE);

    level++;
  } while (level < 2 && valid_iteration && !found_solution);

  return found_solution;
}

bool OccupancyMap::findNearestOnCell(const ImageFeature& input,
                                     ImageFeature&       nearest)
{
  if (n_surf_features == 0) {
    std::cout
        << "WARNING (findNearest): Trying to find nearest feature on empty map..."
        << std::endl;
    return false;
  }

  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int i = static_cast<int>(std::round(input.pos.x / resolution + .49));
  int j = static_cast<int>(std::round(input.pos.y / resolution + .49));

  // distance checker and calculator
  float min_dist = 1e6;
  float dist;

  for (const auto& feature : (*this)(i, j).surf_features) {
    dist = input.pos.distance(feature.pos);
    if (dist < min_dist) {
      min_dist = dist;
      nearest  = feature;
    }
  }

  return true;
}

}; // namespace vineslam
