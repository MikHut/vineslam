#include "occupancy_map.hpp"
#include <chrono>

namespace vineslam
{

MapLayer::MapLayer(const Parameters& params, const pose& origin_offset)
{
  // Read input parameters
  origin.x   = params.gridmap_origin_x + origin_offset.x;
  origin.y   = params.gridmap_origin_y + origin_offset.y;
  resolution = params.gridmap_resolution;
  width      = params.gridmap_width;
  lenght     = params.gridmap_lenght;

  // Set the grid map size
  int map_size =
      static_cast<int>(std::round((width / resolution) * (lenght / resolution)));
  m_gmap = std::vector<Cell>(map_size);

  // Initialize number of features and landmarks
  n_surf_features   = 0;
  n_landmarks       = 0;
  n_corner_features = 0;
  n_points          = 0;
}

MapLayer::MapLayer(const MapLayer& grid_map)
{
  this->m_gmap            = grid_map.m_gmap;
  this->n_corner_features = grid_map.n_corner_features;
  this->n_planar_features = grid_map.n_planar_features;
  this->n_surf_features   = grid_map.n_surf_features;
  this->n_landmarks       = grid_map.n_landmarks;
  this->n_points          = grid_map.n_points;
  this->resolution        = grid_map.resolution;
  this->origin            = grid_map.origin;
  this->lenght            = grid_map.lenght;
  this->width             = grid_map.width;
}

bool MapLayer::insert(const SemanticFeature& m_landmark,
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

  // Mark cell as occupied in pointer array
  int m_i = i - static_cast<int>(std::round(origin.x / resolution + .49));
  int m_j = j - static_cast<int>(std::round(origin.y / resolution + .49));
  int idx = m_i + m_j * static_cast<int>(std::round(width / resolution + .49));
  landmark_indexes.insert(idx);

  return true;
}

bool MapLayer::insert(const SemanticFeature& m_landmark, const int& id)
{
  // Compute grid coordinates for the floating point Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_landmark.pos.x / resolution + .49));
  int m_j = static_cast<int>(std::round(m_landmark.pos.y / resolution + .49));

  return insert(m_landmark, id, m_i, m_j);
}

bool MapLayer::insert(const ImageFeature& m_feature, const int& i, const int& j)
{
  try {
    check(i, j);
  } catch (char const* msg) {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).surf_features.push_back(m_feature);
  n_surf_features++;

  // Mark cell as occupied in pointer array
  int m_i = i - static_cast<int>(std::round(origin.x / resolution + .49));
  int m_j = j - static_cast<int>(std::round(origin.y / resolution + .49));
  int idx = m_i + m_j * static_cast<int>(std::round(width / resolution + .49));
  surf_indexes.insert(idx);

  return true;
}

bool MapLayer::insert(const ImageFeature& m_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_feature.pos.x / resolution + .49));
  int m_j = static_cast<int>(std::round(m_feature.pos.y / resolution + .49));

  return insert(m_feature, m_i, m_j);
}

bool MapLayer::insert(const Corner& m_feature, const int& i, const int& j)
{
  try {
    check(i, j);
  } catch (char const* msg) {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).corner_features.push_back(m_feature);
  n_corner_features++;

  // Mark cell as occupied in pointer array
  int m_i = i - static_cast<int>(std::round(origin.x / resolution + .49));
  int m_j = j - static_cast<int>(std::round(origin.y / resolution + .49));
  int idx = m_i + m_j * static_cast<int>(std::round(width / resolution + .49));
  corner_indexes.insert(idx);

  return true;
}

bool MapLayer::insert(const Corner& m_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_feature.pos.x / resolution + .49));
  int m_j = static_cast<int>(std::round(m_feature.pos.y / resolution + .49));

  return insert(m_feature, m_i, m_j);
}

bool MapLayer::insert(const Planar& m_feature, const int& i, const int& j)
{
  try {
    check(i, j);
  } catch (char const* msg) {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).planar_features.push_back(m_feature);
  n_planar_features++;

  // Mark cell as occupied in pointer array
  int m_i = i - static_cast<int>(std::round(origin.x / resolution + .49));
  int m_j = j - static_cast<int>(std::round(origin.y / resolution + .49));
  int idx = m_i + m_j * static_cast<int>(std::round(width / resolution + .49));
  planar_indexes.insert(idx);

  return true;
}

bool MapLayer::insert(const Planar& m_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_feature.pos.x / resolution + .49));
  int m_j = static_cast<int>(std::round(m_feature.pos.y / resolution + .49));

  return insert(m_feature, m_i, m_j);
}

bool MapLayer::insert(const point& m_point, const int& i, const int& j)
{
  try {
    check(i, j);
  } catch (char const* msg) {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).points.push_back(m_point);
  n_points++;

  // Mark cell as occupied in pointer array
  int m_i = i - static_cast<int>(std::round(origin.x / resolution + .49));
  int m_j = j - static_cast<int>(std::round(origin.y / resolution + .49));
  int idx = m_i + m_j * static_cast<int>(std::round(width / resolution + .49));
  point_indexes.insert(idx);

  return true;
}

bool MapLayer::insert(const point& m_point)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_point.x / resolution + .49));
  int m_j = static_cast<int>(std::round(m_point.y / resolution + .49));

  return insert(m_point, m_i, m_j);
}

bool MapLayer::update(const SemanticFeature& new_landmark,
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
  std::cout << "WARNING (MapLayer::update): Trying to update Landmark that is "
               "not on the map... "
            << std::endl;
  return false;
}

bool MapLayer::update(const Corner& old_corner, const Corner& new_corner)
{
  // Compute grid coordinates for the floating point old Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(old_corner.pos.x / resolution + .49));
  int m_j = static_cast<int>(std::round(old_corner.pos.y / resolution + .49));

  // Access cell of old corner
  Cell m_cell = (*this)(m_i, m_j);
  // Get all the corner in the given cell
  std::vector<Corner> m_corners = m_cell.corner_features;

  // Find the corner and update it
  for (size_t i = 0; i < m_corners.size(); i++) {
    Corner m_corner = m_corners[i];

    if (m_corner.pos.x == old_corner.pos.x && m_corner.pos.y == old_corner.pos.y &&
        m_corner.pos.z == old_corner.pos.z) {

      // Check if new corner lies on the same cell of the source one
      int new_m_i =
          static_cast<int>(std::round(new_corner.pos.x / resolution + .49));
      int new_m_j =
          static_cast<int>(std::round(new_corner.pos.y / resolution + .49));

      if (new_m_i != m_i || new_m_j != m_j) {
        (*this)(m_i, m_j).corner_features.erase(
            (*this)(m_i, m_j).corner_features.begin() + i);
        insert(new_corner);
      } else {
        (*this)(m_i, m_j).corner_features[i] = new_corner;
      }

      return true;
    }
  }

  std::cout << "WARNING (OcuppancyMap::update): Trying to update a corner that is "
               "not on the map... "
            << std::endl;
  return false;
}

bool MapLayer::update(const Planar& old_planar, const Planar& new_planar)
{
  // Compute grid coordinates for the floating point old Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(old_planar.pos.x / resolution + .49));
  int m_j = static_cast<int>(std::round(old_planar.pos.y / resolution + .49));

  // Access cell of old planar
  Cell m_cell = (*this)(m_i, m_j);
  // Get all the planar in the given cell
  std::vector<Planar> m_planars = m_cell.planar_features;

  // Find the planar and update it
  for (size_t i = 0; i < m_planars.size(); i++) {
    Planar m_planar = m_planars[i];

    if (m_planar.pos.x == old_planar.pos.x && m_planar.pos.y == old_planar.pos.y &&
        m_planar.pos.z == old_planar.pos.z) {

      // Check if new planar lies on the same cell of the source one
      int new_m_i =
          static_cast<int>(std::round(new_planar.pos.x / resolution + .49));
      int new_m_j =
          static_cast<int>(std::round(new_planar.pos.y / resolution + .49));

      if (new_m_i != m_i || new_m_j != m_j) {
        (*this)(m_i, m_j).planar_features.erase(
            (*this)(m_i, m_j).planar_features.begin() + i);
        insert(new_planar);
      } else {
        (*this)(m_i, m_j).planar_features[i] = new_planar;
      }

      return true;
    }
  }

  std::cout << "WARNING (OcuppancyMap::update): Trying to update a planar that is "
               "not on the map... "
            << std::endl;
  return false;
}

bool MapLayer::update(const ImageFeature& old_image_feature,
                      const ImageFeature& new_image_feature)
{
}

void MapLayer::downsampleCorners()
{
  for (const auto& i : corner_indexes) {
    point m_pt(0, 0, 0);
    int   wp;
    for (const auto& corner : m_gmap[i].corner_features) {
      m_pt.x += corner.pos.x;
      m_pt.y += corner.pos.y;
      m_pt.z += corner.pos.z;
    }
    m_pt.x /= static_cast<float>(m_gmap[i].corner_features.size());
    m_pt.y /= static_cast<float>(m_gmap[i].corner_features.size());
    m_pt.z /= static_cast<float>(m_gmap[i].corner_features.size());

    m_gmap[i].corner_features.clear();
    Corner c(m_pt, 0);
    m_gmap[i].corner_features = {c};
  }
}

void MapLayer::downsamplePlanars()
{
  for (const auto& i : planar_indexes) {
    auto size = static_cast<float>(m_gmap[i].planar_features.size());
    if (size == 0)
      continue;
    point m_pt(0, 0, 0);
    int   wp;
    for (const auto& planar : m_gmap[i].planar_features) {
      m_pt.x += planar.pos.x;
      m_pt.y += planar.pos.y;
      m_pt.z += planar.pos.z;
    }
    m_pt.x /= size;
    m_pt.y /= size;
    m_pt.z /= size;

    m_gmap[i].planar_features.clear();
    Planar p(m_pt, 0);
    m_gmap[i].planar_features = {p};
  }
}

bool MapLayer::getAdjacent(const int&         i,
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
            m <= (origin.y + lenght) / resolution && m >= origin.y / resolution &&
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

bool MapLayer::getAdjacent(const float&       i,
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

bool MapLayer::findNearest(const ImageFeature& input,
                           ImageFeature&       nearest,
                           float&              sdist)
{
  if (n_surf_features == 0) {
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
  float ddist = std::numeric_limits<float>::max();
  sdist       = std::numeric_limits<float>::max();
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

      // ------- Use feature descriptor to find correspondences
      // ------- Grid map is used to limit the search space
      for (const auto& feature : (*this)(m_i, m_j).surf_features) {
        std::vector<float> desc   = input.signature;
        std::vector<float> m_desc = feature.signature;

        // Check validity of descriptors data
        if (desc.size() != m_desc.size()) {
          std::cout << "WARNING (findNearest): source and target descriptors have "
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
        if (ssd < ddist) {
          ddist   = ssd;
          nearest = feature;

          sdist = input.pos.distance(feature.pos);
        }
      }
      // ---------------------------------------------------------------------------

    } while (move != DONE);

    level++;
  } while (level < 2 && valid_iteration && !found_solution);

  return found_solution;
}

bool MapLayer::findNearest(const Corner& input, Corner& nearest, float& sdist)
{
  if (n_corner_features == 0) {
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
  sdist = std::numeric_limits<float>::max();
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

      // Found solution if there is any feature in the target cell
      found_solution = found_solution | !(*this)(m_i, m_j).corner_features.empty();

      // ------- Use euclidean distance to find correspondences
      // ------- Grid map is used to limit the search space
      for (const auto& feature : (*this)(m_i, m_j).corner_features) {
        float dist = input.pos.distance(feature.pos);
        if (dist < sdist) {
          sdist   = dist;
          nearest = feature;
        }
      }

      // ---------------------------------------------------------------------------

    } while (move != DONE);

    level++;
  } while (level < 2 && valid_iteration && !found_solution);

  return found_solution;
}

bool MapLayer::findNearest(const Planar& input, Planar& nearest, float& sdist)
{
  if (n_planar_features == 0) {
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
  sdist = std::numeric_limits<float>::max();
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

      // Found solution if there is any feature in the target cell
      found_solution = found_solution | !(*this)(m_i, m_j).planar_features.empty();

      // ------- Use euclidean distance to find correspondences
      // ------- Grid map is used to limit the search space
      for (const auto& feature : (*this)(m_i, m_j).planar_features) {
        float dist = input.pos.distance(feature.pos);
        if (dist < sdist) {
          sdist   = dist;
          nearest = feature;
        }
      }

      // ---------------------------------------------------------------------------

    } while (move != DONE);

    level++;
  } while (level < 2 && valid_iteration && !found_solution);

  return found_solution;
}

bool MapLayer::findNearestOnCell(const ImageFeature& input, ImageFeature& nearest)
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
  float min_dist = std::numeric_limits<float>::max();
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

OccupancyMap::OccupancyMap(const Parameters& params, const pose& origin_offset)
{
  // Read input parameters
  origin.x     = params.gridmap_origin_x + origin_offset.x;
  origin.y     = params.gridmap_origin_y + origin_offset.x;
  origin.z     = params.gridmap_origin_z;
  resolution   = params.gridmap_resolution;
  resolution_z = resolution / 8;
  width        = params.gridmap_width;
  lenght       = params.gridmap_lenght;
  height       = params.gridmap_height;
  zmin         = 0;
  zmax         = static_cast<int>(std::round(height / resolution_z)) - 1;

  // Initialize multi-layer grid map
  float i = origin.z;
  while (i < origin.z + height) {
    int layer_num       = getLayerNumber(i);
    m_layers[layer_num] = MapLayer(params, origin_offset);
    i += resolution_z;
  }

  map_planes = {};
}

OccupancyMap::OccupancyMap(const OccupancyMap& grid_map)
{
  this->m_layers     = grid_map.m_layers;
  this->resolution   = grid_map.resolution;
  this->origin       = grid_map.origin;
  this->lenght       = grid_map.lenght;
  this->width        = grid_map.width;
  this->height       = grid_map.height;
  this->resolution_z = grid_map.resolution_z;
  this->zmin         = grid_map.zmin;
  this->zmax         = grid_map.zmax;
  this->map_planes   = grid_map.map_planes;
}

int OccupancyMap::getLayerNumber(const float& z) const
{
  int layer_num = static_cast<int>(std::round((z - origin.z) / resolution_z));

  layer_num = (layer_num < zmin) ? zmin : layer_num;
  layer_num = (layer_num > zmax) ? zmax : layer_num;

  return layer_num;
}

bool OccupancyMap::insert(const SemanticFeature& m_landmark, const int& id)
{
  return m_layers[getLayerNumber(0)].insert(m_landmark, id);
}

bool OccupancyMap::insert(const ImageFeature& m_feature)
{
  return m_layers[getLayerNumber(m_feature.pos.z)].insert(m_feature);
}

bool OccupancyMap::insert(const Corner& m_feature)
{
  return m_layers[getLayerNumber(m_feature.pos.z)].insert(m_feature);
}

bool OccupancyMap::insert(const Planar& m_feature)
{
  return m_layers[getLayerNumber(m_feature.pos.z)].insert(m_feature);
}

bool OccupancyMap::insert(const point& m_point)
{
  return m_layers[getLayerNumber(m_point.z)].insert(m_point);
}

bool OccupancyMap::update(const SemanticFeature& new_landmark,
                          const int&             id,
                          const float&           i,
                          const float&           j)
{
  return m_layers[getLayerNumber(0)].update(new_landmark, id, i, j);
}

bool OccupancyMap::update(const Corner& old_corner, const Corner& new_corner)
{
  int old_layer_num = getLayerNumber(old_corner.pos.z);
  int new_layer_num = getLayerNumber(new_corner.pos.z);

  if (old_layer_num == new_layer_num) {
    return m_layers[old_layer_num].update(old_corner, new_corner);
  } else {
    // Compute grid coordinates for the floating point old corner location
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int m_i = static_cast<int>(std::round(old_corner.pos.x / resolution + .49));
    int m_j = static_cast<int>(std::round(old_corner.pos.y / resolution + .49));

    // Access cell of old corner
    Cell m_cell = m_layers[old_layer_num](m_i, m_j);
    // Get all the corner in the given cell
    std::vector<Corner> m_corners = m_cell.corner_features;

    // Find the corner and update it
    for (size_t i = 0; i < m_corners.size(); i++) {
      Corner m_corner = m_corners[i];

      if (m_corner.pos.x == old_corner.pos.x && m_corner.pos.y == old_corner.pos.y &&
          m_corner.pos.z == old_corner.pos.z) {

        m_layers[old_layer_num](m_i, m_j).corner_features.erase(
            m_layers[old_layer_num](m_i, m_j).corner_features.begin() + i);

        insert(new_corner);

        return true;
      }
    }
  }

  std::cout << "WARNING (OcuppancyMap::update): Trying to update a corner that is "
               "not on the map... "
            << std::endl;
  return false;
}

bool OccupancyMap::update(const Planar& old_planar, const Planar& new_planar)
{
  int old_layer_num = getLayerNumber(old_planar.pos.z);
  int new_layer_num = getLayerNumber(new_planar.pos.z);

  if (old_layer_num == new_layer_num) {
    return m_layers[old_layer_num].update(old_planar, new_planar);
  } else {
    // Compute grid coordinates for the floating point old planar location
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int m_i = static_cast<int>(std::round(old_planar.pos.x / resolution + .49));
    int m_j = static_cast<int>(std::round(old_planar.pos.y / resolution + .49));

    // Access cell of old planar
    Cell m_cell = m_layers[old_layer_num](m_i, m_j);
    // Get all the planar in the given cell
    std::vector<Planar> m_planars = m_cell.planar_features;

    // Find the planar and update it
    for (size_t i = 0; i < m_planars.size(); i++) {
      Planar m_planar = m_planars[i];

      if (m_planar.pos.x == old_planar.pos.x && m_planar.pos.y == old_planar.pos.y &&
          m_planar.pos.z == old_planar.pos.z) {

        m_layers[old_layer_num](m_i, m_j).planar_features.erase(
            m_layers[old_layer_num](m_i, m_j).planar_features.begin() + i);

        insert(new_planar);

        return true;
      }
    }
  }

  std::cout << "WARNING (OcuppancyMap::update): Trying to update a planar that is "
               "not on the map... "
            << std::endl;
  return false;
}

bool OccupancyMap::update(const ImageFeature& old_image_feature,
                          const ImageFeature& new_image_feature)
{
}

void OccupancyMap::downsampleCorners()
{
  for (auto& mlayer : m_layers) mlayer.second.downsampleCorners();
}

void OccupancyMap::downsamplePlanars()
{
  for (auto& mlayer : m_layers) mlayer.second.downsamplePlanars();
}

bool OccupancyMap::getAdjacent(const float&       x,
                               const float&       y,
                               const float&       z,
                               const int&         layers,
                               std::vector<Cell>& adjacent)
{
  // Set up data needed to compute the routine
  std::vector<Cell> up_cells;
  std::vector<Cell> layer_cells;
  std::vector<Cell> down_cells;

  int layer_num = getLayerNumber(z);

  // Find the adjacent cells from the layer, and the upward and downward layers
  if (layer_num - 1 > zmin)
    m_layers[layer_num - 1].getAdjacent(x, y, layers, down_cells);
  if (layer_num + 1 < zmax)
    m_layers[layer_num + 1].getAdjacent(x, y, layers, up_cells);
  if (layer_num > zmin && layer_num < zmax)
    m_layers[layer_num].getAdjacent(x, y, layers, layer_cells);

  // Insert all the obtained cells in the output array
  adjacent.insert(adjacent.end(), down_cells.begin(), down_cells.end());
  adjacent.insert(adjacent.end(), layer_cells.begin(), layer_cells.end());
  adjacent.insert(adjacent.end(), up_cells.begin(), up_cells.end());

  return !adjacent.empty();
}

bool OccupancyMap::findNearest(const ImageFeature& input,
                               ImageFeature&       nearest,
                               float&              sdist)
{
  // Set up data needed to compute the routine
  ImageFeature nearest_down, nearest_up, nearest_layer;
  float        sdist_down = 1e6, sdist_up = 1e6, sdist_layer = 1e6;

  int layer_num = getLayerNumber(input.pos.z);

  // Find the nearest feature in each layer
  bool c1 = m_layers[layer_num - 1].findNearest(input, nearest_down, sdist_down);
  bool c2 = m_layers[layer_num + 1].findNearest(input, nearest_up, sdist_up);
  bool c3 = m_layers[layer_num].findNearest(input, nearest_layer, sdist_layer);

  if (sdist_down < sdist_up && sdist_down < sdist_layer) {
    nearest = nearest_down;
    sdist   = sdist_down;
  } else if (sdist_up < sdist_down && sdist_up < sdist_layer) {
    nearest = nearest_up;
    sdist   = sdist_up;
  } else {
    nearest = nearest_layer;
    sdist   = sdist_layer;
  }

  return c1 || c2 || c3;
}

bool OccupancyMap::findNearest(const Corner& input, Corner& nearest, float& sdist)
{
  // Set up data needed to compute the routine
  Corner nearest_down, nearest_up, nearest_layer;
  float  sdist_down = 1e6, sdist_up = 1e6, sdist_layer = 1e6;

  int layer_num = getLayerNumber(input.pos.z);

  // Find the nearest feature in each layer
  bool c1 = m_layers[layer_num - 1].findNearest(input, nearest_down, sdist_down);
  bool c2 = m_layers[layer_num + 1].findNearest(input, nearest_up, sdist_up);
  bool c3 = m_layers[layer_num].findNearest(input, nearest_layer, sdist_layer);

  if (sdist_down < sdist_up && sdist_down < sdist_layer) {
    nearest = nearest_down;
    sdist   = sdist_down;
  } else if (sdist_up < sdist_down && sdist_up < sdist_layer) {
    nearest = nearest_up;
    sdist   = sdist_up;
  } else {
    nearest = nearest_layer;
    sdist   = sdist_layer;
  }

  return c1 || c2 || c3;
}

bool OccupancyMap::findNearest(const Planar& input, Planar& nearest, float& sdist)
{
  // Set up data needed to compute the routine
  Planar nearest_down, nearest_up, nearest_layer;
  float  sdist_down = 1e6, sdist_up = 1e6, sdist_layer = 1e6;

  int layer_num = getLayerNumber(input.pos.z);

  // Find the nearest feature in each layer
  bool c1 = m_layers[layer_num - 1].findNearest(input, nearest_down, sdist_down);
  bool c2 = m_layers[layer_num + 1].findNearest(input, nearest_up, sdist_up);
  bool c3 = m_layers[layer_num].findNearest(input, nearest_layer, sdist_layer);

  if (sdist_down < sdist_up && sdist_down < sdist_layer) {
    nearest = nearest_down;
    sdist   = sdist_down;
  } else if (sdist_up < sdist_down && sdist_up < sdist_layer) {
    nearest = nearest_up;
    sdist   = sdist_up;
  } else {
    nearest = nearest_layer;
    sdist   = sdist_layer;
  }

  return c1 || c2 || c3;
}

bool OccupancyMap::findNearestOnCell(const ImageFeature& input,
                                     ImageFeature&       nearest)
{
  return m_layers[getLayerNumber(input.pos.z)].findNearestOnCell(input, nearest);
}

} // namespace vineslam
