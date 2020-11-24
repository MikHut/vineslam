#include "../../include/vineslam/mapping/occupancy_map.hpp"

namespace vineslam
{
MapLayer::MapLayer(const Parameters& params, const Pose& origin_offset)
{
  // Read input parameters
  origin_.x_ = params.gridmap_origin_x_ + origin_offset.x_;
  origin_.y_ = params.gridmap_origin_y_ + origin_offset.y_;
  resolution_ = params.gridmap_resolution_;
  width_ = params.gridmap_width_;
  lenght_ = params.gridmap_lenght_;

  // Set the grid map size
  int map_size = static_cast<int>(std::round((width_ / resolution_) * (lenght_ / resolution_)));
  cell_vec_ = std::vector<Cell>(map_size);

  // Initialize number of features and landmarks
  n_surf_features_ = 0;
  n_landmarks_ = 0;
  n_corner_features_ = 0;
  n_points_ = 0;
}

MapLayer::MapLayer(const MapLayer& grid_map)
{
  this->cell_vec_ = grid_map.cell_vec_;
  this->n_corner_features_ = grid_map.n_corner_features_;
  this->n_planar_features_ = grid_map.n_planar_features_;
  this->n_surf_features_ = grid_map.n_surf_features_;
  this->n_landmarks_ = grid_map.n_landmarks_;
  this->n_points_ = grid_map.n_points_;
  this->resolution_ = grid_map.resolution_;
  this->origin_ = grid_map.origin_;
  this->lenght_ = grid_map.lenght_;
  this->width_ = grid_map.width_;
}

bool MapLayer::insert(const SemanticFeature& m_landmark, const int& id, const int& i, const int& j)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).insert(id, m_landmark);
  n_landmarks_++;

  // Mark cell as occupied in pointer array
  int m_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
  int m_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
  int idx = m_i + m_j * static_cast<int>(std::round(width_ / resolution_ + .49));
  landmark_set_.insert(idx);

  return true;
}

bool MapLayer::insert(const SemanticFeature& m_landmark, const int& id)
{
  // Compute grid coordinates for the floating point Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_landmark.pos_.x_ / resolution_ + .49));
  int m_j = static_cast<int>(std::round(m_landmark.pos_.y_ / resolution_ + .49));

  return insert(m_landmark, id, m_i, m_j);
}

bool MapLayer::insert(const ImageFeature& m_feature, const int& i, const int& j)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).surf_features_.push_back(m_feature);
  n_surf_features_++;

  // Mark cell as occupied in pointer array
  int m_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
  int m_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
  int idx = m_i + m_j * static_cast<int>(std::round(width_ / resolution_ + .49));
  surf_set_.insert(idx);

  return true;
}

bool MapLayer::insert(const ImageFeature& m_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_feature.pos_.x_ / resolution_ + .49));
  int m_j = static_cast<int>(std::round(m_feature.pos_.y_ / resolution_ + .49));

  return insert(m_feature, m_i, m_j);
}

bool MapLayer::insert(const Corner& m_feature, const int& i, const int& j)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).corner_features_.push_back(m_feature);
  n_corner_features_++;

  // Mark cell as occupied in pointer array
  int m_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
  int m_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
  int idx = m_i + m_j * static_cast<int>(std::round(width_ / resolution_ + .49));
  corner_set_.insert(idx);

  return true;
}

bool MapLayer::insert(const Corner& m_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_feature.pos_.x_ / resolution_ + .49));
  int m_j = static_cast<int>(std::round(m_feature.pos_.y_ / resolution_ + .49));

  return insert(m_feature, m_i, m_j);
}

bool MapLayer::insert(const Planar& m_feature, const int& i, const int& j)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).planar_features_.push_back(m_feature);
  n_planar_features_++;

  // Mark cell as occupied in pointer array
  int m_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
  int m_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
  int idx = m_i + m_j * static_cast<int>(std::round(width_ / resolution_ + .49));
  planar_set_.insert(idx);

  return true;
}

bool MapLayer::insert(const Planar& m_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_feature.pos_.x_ / resolution_ + .49));
  int m_j = static_cast<int>(std::round(m_feature.pos_.y_ / resolution_ + .49));

  return insert(m_feature, m_i, m_j);
}

bool MapLayer::insert(const Point& m_point, const int& i, const int& j)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
    std::cout << msg;
    return false;
  }

  (*this)(i, j).points_.push_back(m_point);
  n_points_++;

  // Mark cell as occupied in pointer array
  int m_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
  int m_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
  int idx = m_i + m_j * static_cast<int>(std::round(width_ / resolution_ + .49));
  point_set_.insert(idx);

  return true;
}

bool MapLayer::insert(const Point& m_point)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(m_point.x_ / resolution_ + .49));
  int m_j = static_cast<int>(std::round(m_point.y_ / resolution_ + .49));

  return insert(m_point, m_i, m_j);
}

bool MapLayer::update(const SemanticFeature& new_landmark, const int& id, const float& i, const float& j)
{
  // Compute grid coordinates for the floating point old Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(i / resolution_ + .49));
  int m_j = static_cast<int>(std::round(j / resolution_ + .49));

  // Get array of landmarks present in the cell of the input landmark
  Cell m_cell = (*this)(m_i, m_j);
  std::map<int, SemanticFeature> m_landmarks = m_cell.landmarks_;

  // Search for a correspondence
  for (const auto& m_landmark : m_landmarks)
  {
    if (m_landmark.first == id)
    {
      // Update the correspondence to the new landmark and leave the routine
      // - check if the new landmark position matches a different cell in relation
      // with previous position
      // - if so, remove the landmark from the previous cell and insert it in the
      // new correct one
      // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
      int new_m_i = static_cast<int>(std::round(new_landmark.pos_.x_ / resolution_ + .49));
      int new_m_j = static_cast<int>(std::round(new_landmark.pos_.y_ / resolution_ + .49));
      if (new_m_i != m_i || new_m_j != m_j)
      {
        (*this)(m_i, m_j).landmarks_.erase(id);
        insert(new_landmark, id);
      }
      else
      {
        (*this)(m_i, m_j).landmarks_[id] = new_landmark;
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
  int m_i = static_cast<int>(std::round(old_corner.pos_.x_ / resolution_ + .49));
  int m_j = static_cast<int>(std::round(old_corner.pos_.y_ / resolution_ + .49));

  // Access cell of old corner
  Cell m_cell = (*this)(m_i, m_j);
  // Get all the corner in the given cell
  std::vector<Corner> m_corners = m_cell.corner_features_;

  // Find the corner and update it
  for (size_t i = 0; i < m_corners.size(); i++)
  {
    Corner m_corner = m_corners[i];

    if (m_corner.pos_.x_ == old_corner.pos_.x_ && m_corner.pos_.y_ == old_corner.pos_.y_ &&
        m_corner.pos_.z_ == old_corner.pos_.z_)
    {
      // Check if new corner lies on the same cell of the source one
      int new_m_i = static_cast<int>(std::round(new_corner.pos_.x_ / resolution_ + .49));
      int new_m_j = static_cast<int>(std::round(new_corner.pos_.y_ / resolution_ + .49));

      if (new_m_i != m_i || new_m_j != m_j)
      {
        (*this)(m_i, m_j).corner_features_.erase((*this)(m_i, m_j).corner_features_.begin() + i);
        insert(new_corner);
      }
      else
      {
        (*this)(m_i, m_j).corner_features_[i] = new_corner;
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
  int m_i = static_cast<int>(std::round(old_planar.pos_.x_ / resolution_ + .49));
  int m_j = static_cast<int>(std::round(old_planar.pos_.y_ / resolution_ + .49));

  // Access cell of old planar
  Cell m_cell = (*this)(m_i, m_j);
  // Get all the planar in the given cell
  std::vector<Planar> m_planars = m_cell.planar_features_;

  // Find the planar and update it
  for (size_t i = 0; i < m_planars.size(); i++)
  {
    Planar m_planar = m_planars[i];

    if (m_planar.pos_.x_ == old_planar.pos_.x_ && m_planar.pos_.y_ == old_planar.pos_.y_ &&
        m_planar.pos_.z_ == old_planar.pos_.z_)
    {
      // Check if new planar lies on the same cell of the source one
      int new_m_i = static_cast<int>(std::round(new_planar.pos_.x_ / resolution_ + .49));
      int new_m_j = static_cast<int>(std::round(new_planar.pos_.y_ / resolution_ + .49));

      if (new_m_i != m_i || new_m_j != m_j)
      {
        (*this)(m_i, m_j).planar_features_.erase((*this)(m_i, m_j).planar_features_.begin() + i);
        insert(new_planar);
      }
      else
      {
        (*this)(m_i, m_j).planar_features_[i] = new_planar;
      }

      return true;
    }
  }

  std::cout << "WARNING (OcuppancyMap::update): Trying to update a planar that is "
               "not on the map... "
            << std::endl;
  return false;
}

bool MapLayer::update(const ImageFeature& old_image_feature, const ImageFeature& new_image_feature)
{
}

void MapLayer::downsampleCorners()
{
  for (const auto& i : corner_set_)
  {
    Point m_pt(0, 0, 0);
    int wp;
    for (const auto& corner : cell_vec_[i].corner_features_)
    {
      m_pt.x_ += corner.pos_.x_;
      m_pt.y_ += corner.pos_.y_;
      m_pt.z_ += corner.pos_.z_;
    }
    m_pt.x_ /= static_cast<float>(cell_vec_[i].corner_features_.size());
    m_pt.y_ /= static_cast<float>(cell_vec_[i].corner_features_.size());
    m_pt.z_ /= static_cast<float>(cell_vec_[i].corner_features_.size());

    cell_vec_[i].corner_features_.clear();
    Corner c(m_pt, 0);
    cell_vec_[i].corner_features_ = { c };
  }
}

void MapLayer::downsamplePlanars()
{
  for (const auto& i : planar_set_)
  {
    auto size = static_cast<float>(cell_vec_[i].planar_features_.size());
    if (size == 0)
      continue;
    Point m_pt(0, 0, 0);
    int wp;
    for (const auto& planar : cell_vec_[i].planar_features_)
    {
      m_pt.x_ += planar.pos_.x_;
      m_pt.y_ += planar.pos_.y_;
      m_pt.z_ += planar.pos_.z_;
    }
    m_pt.x_ /= size;
    m_pt.y_ /= size;
    m_pt.z_ /= size;

    cell_vec_[i].planar_features_.clear();
    Planar p(m_pt, 0);
    cell_vec_[i].planar_features_ = { p };
  }
}

bool MapLayer::getAdjacent(const int& i, const int& j, const int& layers, std::vector<Cell>& adjacent)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
    std::cout << msg;
    return false;
  }

  if (layers == 1 || layers == 2)
  {
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
    for (int n = i_min; n <= i_max; n++)
    {
      for (int m = j_min; m <= j_max; m++)
      {
        if (n <= (origin_.x_ + width_) / resolution_ && n >= origin_.x_ / resolution_ &&
            m <= (origin_.y_ + lenght_) / resolution_ && m >= origin_.y_ / resolution_ && !(n == i && m == j))
        {
          adjacent[idx] = (*this)(n, m);
          idx++;
        }
        else
        {
          adjacent[idx] = Cell();
          continue;
        }
      }
    }
    return true;
  }
  else
  {
    std::cout << "WARNING: Invalid number of adjacent layers. Only 1 or 2 adjacent "
                 "layers are supported.";
    return false;
  }
}

bool MapLayer::getAdjacent(const float& i, const float& j, const int& layers, std::vector<Cell>& adjacent)
{
  // Compute grid coordinates for the floating point Feature/Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int m_i = static_cast<int>(std::round(i / resolution_ + .49));
  int m_j = static_cast<int>(std::round(j / resolution_ + .49));

  return getAdjacent(m_i, m_j, layers, adjacent);
}

bool MapLayer::findNearest(const ImageFeature& input, ImageFeature& nearest, float& sdist)
{
  if (n_surf_features_ == 0)
  {
    return false;
  }

  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int i = static_cast<int>(std::round(input.pos_.x_ / resolution_ + .49));
  int j = static_cast<int>(std::round(input.pos_.y_ / resolution_ + .49));

  // Enumerator used to go through the nearest neighbor search
  enum moves
  {
    ORIGIN,
    RIGHT,
    DOWN,
    LEFT,
    UP,
    DONE
  };
  moves move = ORIGIN;

  // Target cell current index
  int m_i, m_j;
  // level of search (level = 1 means searching on adjacent cells)
  int level = 0;
  // iterator to move into the desired next cell
  int it = 0;
  // distance checker
  float ddist = std::numeric_limits<float>::max();
  sdist = std::numeric_limits<float>::max();
  // booleans for stop criteria
  bool found_solution;
  bool valid_iteration;

  do
  {
    valid_iteration = false;
    found_solution = false;
    do
    {
      switch (move)
      {
        case ORIGIN:
          try
          {
            check(i, j);
          }
          catch (char const* msg)
          {
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
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            move = DOWN;
            it = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Found solution if there is any feature in the target cell
          if (m_i == i + level)
          {
            move = DOWN;
            it = 1;
          }
          else
          {
            it++;
          }
          break;
        case DOWN:
          // Compute cell indexes
          m_i = i + level;
          m_j = j + level - it;
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            move = LEFT;
            it = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (m_j == j - level)
          {
            move = LEFT;
            it = 1;
          }
          else
          {
            it++;
          }
          break;
        case LEFT:
          // Compute cell indexes
          m_i = i + level - it;
          m_j = j - level;
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            move = UP;
            it = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (m_i == i - level)
          {
            move = UP;
            it = 1;
          }
          else
          {
            it++;
          }
          break;
        case UP:
          // Compute cell indexes
          m_i = i - level;
          m_j = j - level + it;
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            it = 0;
            move = DONE;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          // The '-1' is to not repeat the first iterator (started on RIGHT)
          if (m_j == j + level - 1)
          {
            move = DONE;
            it = 0;
          }
          else
          {
            it++;
          }
          break;
        case DONE:
          move = RIGHT;
          it = 0;
          continue;
      }

      // ------- Use feature descriptor to find correspondences
      // ------- Grid map is used to limit the search space
      for (const auto& feature : (*this)(m_i, m_j).surf_features_)
      {
        std::vector<float> desc = input.signature_;
        std::vector<float> m_desc = feature.signature_;

        // Check validity of descriptors data
        if (desc.size() != m_desc.size())
        {
          std::cout << "WARNING (findNearest): source and target descriptors have "
                       "different size ... "
                    << std::endl;
          break;
        }

        // Check if source and target features are of the same type
        if (feature.laplacian_ != input.laplacian_)
          continue;

        // Found solution if there is any correspondence between features of the
        // same type
        found_solution = true;

        float ssd = 0.;  // sum of square errors
        for (size_t k = 0; k < desc.size(); k++)
          ssd += (desc[k] - m_desc[k]) * (desc[k] - m_desc[k]);

        // Update correspondence if we found a local minimum
        if (ssd < ddist)
        {
          ddist = ssd;
          nearest = feature;

          sdist = input.pos_.distance(feature.pos_);
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
  if (n_corner_features_ == 0)
  {
    return false;
  }

  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int i = static_cast<int>(std::round(input.pos_.x_ / resolution_ + .49));
  int j = static_cast<int>(std::round(input.pos_.y_ / resolution_ + .49));

  // Enumerator used to go through the nearest neighbor search
  enum moves
  {
    ORIGIN,
    RIGHT,
    DOWN,
    LEFT,
    UP,
    DONE
  };
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

  do
  {
    valid_iteration = false;
    found_solution = false;
    do
    {
      switch (move)
      {
        case ORIGIN:
          try
          {
            check(i, j);
          }
          catch (char const* msg)
          {
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
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            move = DOWN;
            it = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Found solution if there is any feature in the target cell
          if (m_i == i + level)
          {
            move = DOWN;
            it = 1;
          }
          else
          {
            it++;
          }
          break;
        case DOWN:
          // Compute cell indexes
          m_i = i + level;
          m_j = j + level - it;
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            move = LEFT;
            it = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (m_j == j - level)
          {
            move = LEFT;
            it = 1;
          }
          else
          {
            it++;
          }
          break;
        case LEFT:
          // Compute cell indexes
          m_i = i + level - it;
          m_j = j - level;
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            move = UP;
            it = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (m_i == i - level)
          {
            move = UP;
            it = 1;
          }
          else
          {
            it++;
          }
          break;
        case UP:
          // Compute cell indexes
          m_i = i - level;
          m_j = j - level + it;
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            it = 0;
            move = DONE;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          // The '-1' is to not repeat the first iterator (started on RIGHT)
          if (m_j == j + level - 1)
          {
            move = DONE;
            it = 0;
          }
          else
          {
            it++;
          }
          break;
        case DONE:
          move = RIGHT;
          it = 0;
          continue;
      }

      // Found solution if there is any feature in the target cell
      found_solution = found_solution | !(*this)(m_i, m_j).corner_features_.empty();

      // ------- Use euclidean distance to find correspondences
      // ------- Grid map is used to limit the search space
      for (const auto& feature : (*this)(m_i, m_j).corner_features_)
      {
        float dist = input.pos_.distance(feature.pos_);
        if (dist < sdist)
        {
          sdist = dist;
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
  if (n_planar_features_ == 0)
  {
    return false;
  }

  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int i = static_cast<int>(std::round(input.pos_.x_ / resolution_ + .49));
  int j = static_cast<int>(std::round(input.pos_.y_ / resolution_ + .49));

  // Enumerator used to go through the nearest neighbor search
  enum moves
  {
    ORIGIN,
    RIGHT,
    DOWN,
    LEFT,
    UP,
    DONE
  };
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

  do
  {
    valid_iteration = false;
    found_solution = false;
    do
    {
      switch (move)
      {
        case ORIGIN:
          try
          {
            check(i, j);
          }
          catch (char const* msg)
          {
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
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            move = DOWN;
            it = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Found solution if there is any feature in the target cell
          if (m_i == i + level)
          {
            move = DOWN;
            it = 1;
          }
          else
          {
            it++;
          }
          break;
        case DOWN:
          // Compute cell indexes
          m_i = i + level;
          m_j = j + level - it;
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            move = LEFT;
            it = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (m_j == j - level)
          {
            move = LEFT;
            it = 1;
          }
          else
          {
            it++;
          }
          break;
        case LEFT:
          // Compute cell indexes
          m_i = i + level - it;
          m_j = j - level;
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            move = UP;
            it = 1;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (m_i == i - level)
          {
            move = UP;
            it = 1;
          }
          else
          {
            it++;
          }
          break;
        case UP:
          // Compute cell indexes
          m_i = i - level;
          m_j = j - level + it;
          try
          {
            check(m_i, m_j);
          }
          catch (char const* msg)
          {
            it = 0;
            move = DONE;
            continue;
          }

          // The iteration is valid since (m_i, m_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          // The '-1' is to not repeat the first iterator (started on RIGHT)
          if (m_j == j + level - 1)
          {
            move = DONE;
            it = 0;
          }
          else
          {
            it++;
          }
          break;
        case DONE:
          move = RIGHT;
          it = 0;
          continue;
      }

      // Found solution if there is any feature in the target cell
      found_solution = found_solution | !(*this)(m_i, m_j).planar_features_.empty();

      // ------- Use euclidean distance to find correspondences
      // ------- Grid map is used to limit the search space
      for (const auto& feature : (*this)(m_i, m_j).planar_features_)
      {
        float dist = input.pos_.distance(feature.pos_);
        if (dist < sdist)
        {
          sdist = dist;
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
  if (n_surf_features_ == 0)
  {
    std::cout << "WARNING (findNearest): Trying to find nearest feature on empty map..." << std::endl;
    return false;
  }

  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int i = static_cast<int>(std::round(input.pos_.x_ / resolution_ + .49));
  int j = static_cast<int>(std::round(input.pos_.y_ / resolution_ + .49));

  // distance checker and calculator
  float min_dist = std::numeric_limits<float>::max();
  float dist;

  for (const auto& feature : (*this)(i, j).surf_features_)
  {
    dist = input.pos_.distance(feature.pos_);
    if (dist < min_dist)
    {
      min_dist = dist;
      nearest = feature;
    }
  }

  return true;
}

OccupancyMap::OccupancyMap(const Parameters& params, const Pose& origin_offset)
{
  // Read input parameters
  origin_.x_ = params.gridmap_origin_x_ + origin_offset.x_;
  origin_.y_ = params.gridmap_origin_y_ + origin_offset.x_;
  origin_.z_ = params.gridmap_origin_z_;
  resolution_ = params.gridmap_resolution_;
  resolution_z_ = resolution_ / 8;
  width_ = params.gridmap_width_;
  lenght_ = params.gridmap_lenght_;
  height_ = params.gridmap_height_;
  zmin_ = 0;
  zmax_ = static_cast<int>(std::round(height_ / resolution_z_)) - 1;

  // Initialize multi-layer grid map
  float i = origin_.z_;
  while (i < origin_.z_ + height_)
  {
    int layer_num = getLayerNumber(i);
    layers_map_[layer_num] = MapLayer(params, origin_offset);
    i += resolution_z_;
  }
}

OccupancyMap::OccupancyMap(const OccupancyMap& grid_map)
{
  this->layers_map_ = grid_map.layers_map_;
  this->resolution_ = grid_map.resolution_;
  this->origin_ = grid_map.origin_;
  this->lenght_ = grid_map.lenght_;
  this->width_ = grid_map.width_;
  this->height_ = grid_map.height_;
  this->resolution_z_ = grid_map.resolution_z_;
  this->zmin_ = grid_map.zmin_;
  this->zmax_ = grid_map.zmax_;
}

int OccupancyMap::getLayerNumber(const float& z) const
{
  int layer_num = static_cast<int>(std::round((z - origin_.z_) / resolution_z_));

  layer_num = (layer_num < zmin_) ? zmin_ : layer_num;
  layer_num = (layer_num > zmax_) ? zmax_ : layer_num;

  return layer_num;
}

bool OccupancyMap::insert(const SemanticFeature& m_landmark, const int& id)
{
  return layers_map_[getLayerNumber(0)].insert(m_landmark, id);
}

bool OccupancyMap::insert(const ImageFeature& m_feature)
{
  return layers_map_[getLayerNumber(m_feature.pos_.z_)].insert(m_feature);
}

bool OccupancyMap::insert(const Corner& m_feature)
{
  return layers_map_[getLayerNumber(m_feature.pos_.z_)].insert(m_feature);
}

bool OccupancyMap::insert(const Planar& m_feature)
{
  return layers_map_[getLayerNumber(m_feature.pos_.z_)].insert(m_feature);
}

bool OccupancyMap::insert(const Point& m_point)
{
  return layers_map_[getLayerNumber(m_point.z_)].insert(m_point);
}

bool OccupancyMap::update(const SemanticFeature& new_landmark, const int& id, const float& i, const float& j)
{
  return layers_map_[getLayerNumber(0)].update(new_landmark, id, i, j);
}

bool OccupancyMap::update(const Corner& old_corner, const Corner& new_corner)
{
  int old_layer_num = getLayerNumber(old_corner.pos_.z_);
  int new_layer_num = getLayerNumber(new_corner.pos_.z_);

  if (old_layer_num == new_layer_num)
  {
    return layers_map_[old_layer_num].update(old_corner, new_corner);
  }
  else
  {
    // Compute grid coordinates for the floating point old corner location
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int m_i = static_cast<int>(std::round(old_corner.pos_.x_ / resolution_ + .49));
    int m_j = static_cast<int>(std::round(old_corner.pos_.y_ / resolution_ + .49));

    // Access cell of old corner
    Cell m_cell = layers_map_[old_layer_num](m_i, m_j);
    // Get all the corner in the given cell
    std::vector<Corner> m_corners = m_cell.corner_features_;

    // Find the corner and update it
    for (size_t i = 0; i < m_corners.size(); i++)
    {
      Corner m_corner = m_corners[i];

      if (m_corner.pos_.x_ == old_corner.pos_.x_ && m_corner.pos_.y_ == old_corner.pos_.y_ &&
          m_corner.pos_.z_ == old_corner.pos_.z_)
      {
        layers_map_[old_layer_num](m_i, m_j).corner_features_.erase(
            layers_map_[old_layer_num](m_i, m_j).corner_features_.begin() + i);

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
  int old_layer_num = getLayerNumber(old_planar.pos_.z_);
  int new_layer_num = getLayerNumber(new_planar.pos_.z_);

  if (old_layer_num == new_layer_num)
  {
    return layers_map_[old_layer_num].update(old_planar, new_planar);
  }
  else
  {
    // Compute grid coordinates for the floating point old planar location
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int m_i = static_cast<int>(std::round(old_planar.pos_.x_ / resolution_ + .49));
    int m_j = static_cast<int>(std::round(old_planar.pos_.y_ / resolution_ + .49));

    // Access cell of old planar
    Cell m_cell = layers_map_[old_layer_num](m_i, m_j);
    // Get all the planar in the given cell
    std::vector<Planar> m_planars = m_cell.planar_features_;

    // Find the planar and update it
    for (size_t i = 0; i < m_planars.size(); i++)
    {
      Planar m_planar = m_planars[i];

      if (m_planar.pos_.x_ == old_planar.pos_.x_ && m_planar.pos_.y_ == old_planar.pos_.y_ &&
          m_planar.pos_.z_ == old_planar.pos_.z_)
      {
        layers_map_[old_layer_num](m_i, m_j).planar_features_.erase(
            layers_map_[old_layer_num](m_i, m_j).planar_features_.begin() + i);

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

bool OccupancyMap::update(const ImageFeature& old_image_feature, const ImageFeature& new_image_feature)
{
}

void OccupancyMap::downsampleCorners()
{
  for (auto& mlayer : layers_map_)
    mlayer.second.downsampleCorners();
}

void OccupancyMap::downsamplePlanars()
{
  for (auto& mlayer : layers_map_)
    mlayer.second.downsamplePlanars();
}

bool OccupancyMap::getAdjacent(const float& x, const float& y, const float& z, const int& layers,
                               std::vector<Cell>& adjacent)
{
  // Set up data needed to compute the routine
  std::vector<Cell> up_cells;
  std::vector<Cell> layer_cells;
  std::vector<Cell> down_cells;

  int layer_num = getLayerNumber(z);

  // Find the adjacent cells from the layer, and the upward and downward layers
  if (layer_num - 1 > zmin_)
    layers_map_[layer_num - 1].getAdjacent(x, y, layers, down_cells);
  if (layer_num + 1 < zmax_)
    layers_map_[layer_num + 1].getAdjacent(x, y, layers, up_cells);
  if (layer_num > zmin_ && layer_num < zmax_)
    layers_map_[layer_num].getAdjacent(x, y, layers, layer_cells);

  // Insert all the obtained cells in the output array
  adjacent.insert(adjacent.end(), down_cells.begin(), down_cells.end());
  adjacent.insert(adjacent.end(), layer_cells.begin(), layer_cells.end());
  adjacent.insert(adjacent.end(), up_cells.begin(), up_cells.end());

  return !adjacent.empty();
}

bool OccupancyMap::findNearest(const ImageFeature& input, ImageFeature& nearest, float& sdist)
{
  // Set up data needed to compute the routine
  ImageFeature nearest_down, nearest_up, nearest_layer;
  float sdist_down = 1e6, sdist_up = 1e6, sdist_layer = 1e6;

  int layer_num = getLayerNumber(input.pos_.z_);

  // Find the nearest feature in each layer
  bool c1 = layers_map_[layer_num - 1].findNearest(input, nearest_down, sdist_down);
  bool c2 = layers_map_[layer_num + 1].findNearest(input, nearest_up, sdist_up);
  bool c3 = layers_map_[layer_num].findNearest(input, nearest_layer, sdist_layer);

  if (sdist_down < sdist_up && sdist_down < sdist_layer)
  {
    nearest = nearest_down;
    sdist = sdist_down;
  }
  else if (sdist_up < sdist_down && sdist_up < sdist_layer)
  {
    nearest = nearest_up;
    sdist = sdist_up;
  }
  else
  {
    nearest = nearest_layer;
    sdist = sdist_layer;
  }

  return c1 || c2 || c3;
}

bool OccupancyMap::findNearest(const Corner& input, Corner& nearest, float& sdist)
{
  // Set up data needed to compute the routine
  Corner nearest_down, nearest_up, nearest_layer;
  float sdist_down = 1e6, sdist_up = 1e6, sdist_layer = 1e6;

  int layer_num = getLayerNumber(input.pos_.z_);

  // Find the nearest feature in each layer
  bool c1 = layers_map_[layer_num - 1].findNearest(input, nearest_down, sdist_down);
  bool c2 = layers_map_[layer_num + 1].findNearest(input, nearest_up, sdist_up);
  bool c3 = layers_map_[layer_num].findNearest(input, nearest_layer, sdist_layer);

  if (sdist_down < sdist_up && sdist_down < sdist_layer)
  {
    nearest = nearest_down;
    sdist = sdist_down;
  }
  else if (sdist_up < sdist_down && sdist_up < sdist_layer)
  {
    nearest = nearest_up;
    sdist = sdist_up;
  }
  else
  {
    nearest = nearest_layer;
    sdist = sdist_layer;
  }

  return c1 || c2 || c3;
}

bool OccupancyMap::findNearest(const Planar& input, Planar& nearest, float& sdist)
{
  // Set up data needed to compute the routine
  Planar nearest_down, nearest_up, nearest_layer;
  float sdist_down = 1e6, sdist_up = 1e6, sdist_layer = 1e6;

  int layer_num = getLayerNumber(input.pos_.z_);

  // Find the nearest feature in each layer
  bool c1 = layers_map_[layer_num - 1].findNearest(input, nearest_down, sdist_down);
  bool c2 = layers_map_[layer_num + 1].findNearest(input, nearest_up, sdist_up);
  bool c3 = layers_map_[layer_num].findNearest(input, nearest_layer, sdist_layer);

  if (sdist_down < sdist_up && sdist_down < sdist_layer)
  {
    nearest = nearest_down;
    sdist = sdist_down;
  }
  else if (sdist_up < sdist_down && sdist_up < sdist_layer)
  {
    nearest = nearest_up;
    sdist = sdist_up;
  }
  else
  {
    nearest = nearest_layer;
    sdist = sdist_layer;
  }

  return c1 || c2 || c3;
}

bool OccupancyMap::findNearestOnCell(const ImageFeature& input, ImageFeature& nearest)
{
  return layers_map_[getLayerNumber(input.pos_.z_)].findNearestOnCell(input, nearest);
}

}  // namespace vineslam
