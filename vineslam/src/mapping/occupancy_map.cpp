#include "../../include/vineslam/mapping/occupancy_map.hpp"
#include <sys/resource.h>

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

  // Set the minimum number of corners and planar feature observations to add them to the map
  min_planar_obsvs_ = 20;
  min_corner_obsvs_ = 5;
}

MapLayer::MapLayer(const MapLayer& grid_map)
{
  this->cell_vec_ = grid_map.cell_vec_;
  this->n_corner_features_ = grid_map.n_corner_features_;
  this->n_planar_features_ = grid_map.n_planar_features_;
  this->n_surf_features_ = grid_map.n_surf_features_;
  this->n_landmarks_ = grid_map.n_landmarks_;
  this->resolution_ = grid_map.resolution_;
  this->origin_ = grid_map.origin_;
  this->lenght_ = grid_map.lenght_;
  this->width_ = grid_map.width_;
  this->min_planar_obsvs_ = grid_map.min_planar_obsvs_;
  this->min_corner_obsvs_ = grid_map.min_corner_obsvs_;
}

bool MapLayer::insert(const SemanticFeature& l_landmark, const int& id, const int& i, const int& j)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
    std::cout << "MapLayer::insert(SemanticFeature) --- " << msg;
    return false;
  }

  // Check if cell already has some semantic feature
  if ((*this)(i, j).landmarks_ == nullptr)
  {
    (*this)(i, j).landmarks_ = new std::map<int, SemanticFeature>();
  }

  cellInsert(id, l_landmark, (*this)(i, j).landmarks_);
  n_landmarks_++;

  // Mark cell as occupied in pointer array
  int l_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
  int l_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
  int idx = l_i + l_j * static_cast<int>(std::round(width_ / resolution_ + .49));
  landmark_set_.insert(idx);

  return true;
}

bool MapLayer::insert(const SemanticFeature& l_landmark, const int& id)
{
  // Compute grid coordinates for the floating point Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int l_i = static_cast<int>(std::round(l_landmark.pos_.x_ / resolution_ + .49));
  int l_j = static_cast<int>(std::round(l_landmark.pos_.y_ / resolution_ + .49));

  return insert(l_landmark, id, l_i, l_j);
}

bool MapLayer::insert(const ImageFeature& l_feature, const int& i, const int& j)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
    std::cout << "MapLayer::insert(ImageFeature) --- " << msg;
    return false;
  }

  // Check if cell already has some surf feature
  if ((*this)(i, j).surf_features_ == nullptr)
  {
    (*this)(i, j).surf_features_ = new std::vector<ImageFeature>();
  }

  (*this)(i, j).surf_features_->push_back(l_feature);
  n_surf_features_++;

  // Mark cell as occupied in pointer array
  int l_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
  int l_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
  int idx = l_i + l_j * static_cast<int>(std::round(width_ / resolution_ + .49));
  surf_set_.insert(idx);

  return true;
}

bool MapLayer::insert(const ImageFeature& l_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int l_i = static_cast<int>(std::round(l_feature.pos_.x_ / resolution_ + .49));
  int l_j = static_cast<int>(std::round(l_feature.pos_.y_ / resolution_ + .49));

  return insert(l_feature, l_i, l_j);
}

bool MapLayer::insert(const Corner& l_feature, const int& i, const int& j)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
    std::cout << "MapLayer::insert(Corner) --- " << msg;
    return false;
  }

  // Check if cell already has some corner feature
  if ((*this)(i, j).corner_features_ == nullptr)
  {
    (*this)(i, j).corner_features_ = new std::vector<Corner>();
    (*this)(i, j).candidate_corner_features_ = new std::vector<Corner>();
  }

  if ((*this)(i, j).candidate_corner_features_->size() < min_corner_obsvs_ - 1)  // insert a candidate (not enough
                                                                                 // observations yet)
  {
    (*this)(i, j).candidate_corner_features_->push_back(l_feature);

    return true;
  }
  else if ((*this)(i, j).candidate_corner_features_->size() == min_corner_obsvs_ - 1)  // we reached the minimum
                                                                                       // number of observations
  {
    (*this)(i, j).candidate_corner_features_->push_back(l_feature);

    // Compute the mean of the candidates
    uint32_t n_candidates = (*this)(i, j).candidate_corner_features_->size();
    Point l_pt(0, 0, 0);
    for (uint32_t k = 0; k < n_candidates; ++k)
    {
      Corner c1 = (*(*this)(i, j).candidate_corner_features_)[k];

      l_pt.x_ += c1.pos_.x_;
      l_pt.y_ += c1.pos_.y_;
      l_pt.z_ += c1.pos_.z_;
    }
    l_pt.x_ /= n_candidates;
    l_pt.y_ /= n_candidates;
    l_pt.z_ /= n_candidates;

    // Insert it in the map
    Corner c(l_pt, 0);
    (*this)(i, j).corner_features_->push_back(c);
    n_corner_features_++;
  }
  else if ((*this)(i, j).candidate_corner_features_->size() >= min_corner_obsvs_)  // normal insertion after reaching
                                                                                   // the minimum number of
                                                                                   // observations
  {
    (*this)(i, j).corner_features_->push_back(l_feature);
    n_corner_features_++;
  }

  // Mark cell as occupied in pointer array
  int l_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
  int l_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
  int idx = l_i + l_j * static_cast<int>(std::round(width_ / resolution_ + .49));
  corner_set_.insert(idx);

  return true;
}

bool MapLayer::insert(const Corner& l_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int l_i = static_cast<int>(std::round(l_feature.pos_.x_ / resolution_ + .49));
  int l_j = static_cast<int>(std::round(l_feature.pos_.y_ / resolution_ + .49));

  return insert(l_feature, l_i, l_j);
}

bool MapLayer::insert(const Planar& l_feature, const int& i, const int& j)
{
  try
  {
    check(i, j);
  }
  catch (char const* msg)
  {
    std::cout << "MapLayer::insert(Planar) --- " << msg;
    return false;
  }

  // Check if cell already has some planar feature
  if ((*this)(i, j).planar_features_ == nullptr)
  {
    (*this)(i, j).planar_features_ = new std::vector<Planar>();
    (*this)(i, j).candidate_planar_features_ = new std::vector<Planar>();
  }

  if ((*this)(i, j).candidate_planar_features_->size() < min_planar_obsvs_ - 1)  // insert a candidate (not enough
                                                                                 // observations yet)
  {
    (*this)(i, j).candidate_planar_features_->push_back(l_feature);

    return true;
  }
  else if ((*this)(i, j).candidate_planar_features_->size() == min_planar_obsvs_ - 1)  // we reached the minimum
                                                                                       // number of observations
  {
    (*this)(i, j).candidate_planar_features_->push_back(l_feature);

    // Compute the mean of the candidates
    uint32_t n_candidates = (*this)(i, j).candidate_planar_features_->size();
    Point l_pt(0, 0, 0);
    for (uint32_t k = 0; k < n_candidates; ++k)
    {
      Planar p1 = (*(*this)(i, j).candidate_planar_features_)[k];

      l_pt.x_ += p1.pos_.x_;
      l_pt.y_ += p1.pos_.y_;
      l_pt.z_ += p1.pos_.z_;
    }
    l_pt.x_ /= n_candidates;
    l_pt.y_ /= n_candidates;
    l_pt.z_ /= n_candidates;

    // Insert it in the map
    Planar p(l_pt, 0);
    (*this)(i, j).planar_features_->push_back(p);
    n_planar_features_++;
  }
  else if ((*this)(i, j).candidate_planar_features_->size() >= min_planar_obsvs_)  // normal insertion after reaching
                                                                                   // the minimum number of
                                                                                   // observations
  {
    (*this)(i, j).planar_features_->push_back(l_feature);
    n_planar_features_++;
  }

  // Mark cell as occupied in pointer array
  int l_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
  int l_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
  int idx = l_i + l_j * static_cast<int>(std::round(width_ / resolution_ + .49));
  planar_set_.insert(idx);

  return true;
}

bool MapLayer::insert(const Planar& l_feature)
{
  // Compute grid coordinates for the floating point Feature location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int l_i = static_cast<int>(std::round(l_feature.pos_.x_ / resolution_ + .49));
  int l_j = static_cast<int>(std::round(l_feature.pos_.y_ / resolution_ + .49));

  return insert(l_feature, l_i, l_j);
}

bool MapLayer::update(const SemanticFeature& new_landmark, const int& id, const float& i, const float& j)
{
  // Compute grid coordinates for the floating point old Landmark location
  // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
  int l_i = static_cast<int>(std::round(i / resolution_ + .49));
  int l_j = static_cast<int>(std::round(j / resolution_ + .49));

  // Get array of landmarks present in the cell of the input landmark
  Cell l_cell = (*this)(l_i, l_j);
  std::map<int, SemanticFeature>* l_landmarks = l_cell.landmarks_;

  if (l_landmarks == nullptr)
  {
  }
  else
  {
    for (const auto& l_landmark : *l_landmarks)
    {
      if (l_landmark.first == id)
      {
        // Update the correspondence to the new landmark and leave the routine
        // - check if the new landmark position matches a different cell in relation
        // with previous position
        // - if so, remove the landmark from the previous cell and insert it in the
        // new correct one
        // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
        int new_l_i = static_cast<int>(std::round(new_landmark.pos_.x_ / resolution_ + .49));
        int new_l_j = static_cast<int>(std::round(new_landmark.pos_.y_ / resolution_ + .49));
        if (new_l_i != l_i || new_l_j != l_j)
        {
          (*this)(l_i, l_j).landmarks_->erase(id);
          insert(new_landmark, id);
        }
        else
        {
          (*(*this)(l_i, l_j).landmarks_)[id] = new_landmark;
        }
        return true;
      }
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
  int l_i = static_cast<int>(std::round(old_corner.pos_.x_ / resolution_ + .49));
  int l_j = static_cast<int>(std::round(old_corner.pos_.y_ / resolution_ + .49));

  // Access cell of old corner
  Cell l_cell = (*this)(l_i, l_j);
  // Get all the corner in the given cell
  std::vector<Corner>* l_corners = l_cell.corner_features_;

  if (l_corners == nullptr)
  {
  }
  else
  {
    // Find the corner and update it
    for (size_t i = 0; i < l_corners->size(); i++)
    {
      Corner l_corner = (*l_corners)[i];

      if (l_corner.pos_.x_ == old_corner.pos_.x_ && l_corner.pos_.y_ == old_corner.pos_.y_ &&
          l_corner.pos_.z_ == old_corner.pos_.z_)
      {
        // Check if new corner lies on the same cell of the source one
        int new_l_i = static_cast<int>(std::round(new_corner.pos_.x_ / resolution_ + .49));
        int new_l_j = static_cast<int>(std::round(new_corner.pos_.y_ / resolution_ + .49));

        if (new_l_i != l_i || new_l_j != l_j)
        {
          (*this)(l_i, l_j).corner_features_->erase((*this)(l_i, l_j).corner_features_->begin() + i);
          insert(new_corner);
        }
        else
        {
          (*(*this)(l_i, l_j).corner_features_)[i] = new_corner;
        }

        return true;
      }
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
  int l_i = static_cast<int>(std::round(old_planar.pos_.x_ / resolution_ + .49));
  int l_j = static_cast<int>(std::round(old_planar.pos_.y_ / resolution_ + .49));

  // Access cell of old planar
  Cell l_cell = (*this)(l_i, l_j);
  // Get all the planar in the given cell
  std::vector<Planar>* l_planars = l_cell.planar_features_;

  if (l_planars == nullptr)
  {
  }
  else
  {
    // Find the planar and update it
    for (size_t i = 0; i < l_planars->size(); i++)
    {
      Planar l_planar = (*l_planars)[i];

      if (l_planar.pos_.x_ == old_planar.pos_.x_ && l_planar.pos_.y_ == old_planar.pos_.y_ &&
          l_planar.pos_.z_ == old_planar.pos_.z_)
      {
        // Check if new planar lies on the same cell of the source one
        int new_l_i = static_cast<int>(std::round(new_planar.pos_.x_ / resolution_ + .49));
        int new_l_j = static_cast<int>(std::round(new_planar.pos_.y_ / resolution_ + .49));

        if (new_l_i != l_i || new_l_j != l_j)
        {
          (*this)(l_i, l_j).planar_features_->erase((*this)(l_i, l_j).planar_features_->begin() + i);
          insert(new_planar);
        }
        else
        {
          (*(*this)(l_i, l_j).planar_features_)[i] = new_planar;
        }

        return true;
      }
    }
  }

  std::cout << "WARNING (OcuppancyMap::update): Trying to update a planar that is "
               "not on the map... "
            << std::endl;
  return false;
}

bool MapLayer::update(const ImageFeature& old_image_feature, const ImageFeature& new_image_feature)
{
  return false;
}

void MapLayer::downsampleCorners()
{
  for (const auto& i : corner_set_)
  {
    std::vector<Corner>* l_corners = cell_vec_[i].corner_features_;

    if (l_corners == nullptr)
    {
      continue;
    }

    auto size = static_cast<float>(l_corners->size());
    if (size == 0)
      continue;
    Point l_pt(0, 0, 0);
    int wp;
    for (const auto& corner : *l_corners)
    {
      l_pt.x_ += corner.pos_.x_;
      l_pt.y_ += corner.pos_.y_;
      l_pt.z_ += corner.pos_.z_;
    }
    l_pt.x_ /= size;
    l_pt.y_ /= size;
    l_pt.z_ /= size;

    cell_vec_[i].corner_features_->clear();
    Corner c(l_pt, 0);
    *(cell_vec_[i].corner_features_) = { c };
  }
}

void MapLayer::downsamplePlanars()
{
  for (const auto& i : planar_set_)
  {
    std::vector<Planar>* l_planars = cell_vec_[i].planar_features_;

    if (l_planars == nullptr)
    {
      continue;
    }

    auto size = static_cast<float>(l_planars->size());
    if (size == 0)
      continue;
    Point l_pt(0, 0, 0);
    for (const auto& planar : *l_planars)
    {
      l_pt.x_ += planar.pos_.x_;
      l_pt.y_ += planar.pos_.y_;
      l_pt.z_ += planar.pos_.z_;
    }
    l_pt.x_ /= size;
    l_pt.y_ /= size;
    l_pt.z_ /= size;

    cell_vec_[i].planar_features_->clear();
    Planar p(l_pt, 0);
    *(cell_vec_[i].planar_features_) = { p };
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
    std::cout << "MapLayer::GetAdjacent() --- " << msg;
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
  int l_i = static_cast<int>(std::round(i / resolution_ + .49));
  int l_j = static_cast<int>(std::round(j / resolution_ + .49));

  return getAdjacent(l_i, l_j, layers, adjacent);
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
  int l_i, l_j;
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
            std::cout << "MapLayer::findNearest(ImageFeature) --- " << msg;
            return false;
          }

          // Set cell indexes where to find correspondences
          l_i = i;
          l_j = j;
          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // End search if we found a correspondence in the source cell
          move = DONE;
          break;
        case RIGHT:
          // Compute cell indexes
          l_i = i - level + it;
          l_j = j + level;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            move = DOWN;
            it = 1;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Found solution if there is any feature in the target cell
          if (l_i == i + level)
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
          l_i = i + level;
          l_j = j + level - it;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            move = LEFT;
            it = 1;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (l_j == j - level)
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
          l_i = i + level - it;
          l_j = j - level;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            move = UP;
            it = 1;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (l_i == i - level)
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
          l_i = i - level;
          l_j = j - level + it;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            it = 0;
            move = DONE;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          // The '-1' is to not repeat the first iterator (started on RIGHT)
          if (l_j == j + level - 1)
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

      //      // ------- Use feature descriptor to find correspondences
      //      // ------- Grid map is used to limit the search space
      //      for (const auto& feature : *(*this)(l_i, l_j).surf_features_)
      //      {
      //        std::vector<float> desc = input.signature_;
      //        std::vector<float> l_desc = feature.signature_;
      //
      //        // Check validity of descriptors data
      //        if (desc.size() != l_desc.size())
      //        {
      //          std::cout << "WARNING (findNearest): source and target descriptors have "
      //                       "different size ... "
      //                    << std::endl;
      //          break;
      //        }
      //
      //        // Check if source and target features are of the same type
      //        if (feature.laplacian_ != input.laplacian_)
      //          continue;
      //
      //        // Found solution if there is any correspondence between features of the
      //        // same type
      //        found_solution = true;
      //
      //        float ssd = 0.;  // sum of square errors
      //        for (size_t k = 0; k < desc.size(); k++)
      //          ssd += (desc[k] - l_desc[k]) * (desc[k] - l_desc[k]);
      //
      //        // Update correspondence if we found a local minimum
      //        if (ssd < ddist)
      //        {
      //          ddist = ssd;
      //          nearest = feature;
      //
      //          sdist = input.pos_.distance(feature.pos_);
      //        }
      //      }
      //      // ---------------------------------------------------------------------------

      std::vector<ImageFeature>* l_image_features = (*this)(l_i, l_j).surf_features_;
      if (l_image_features == nullptr)
      {
      }
      else
      {
        found_solution = found_solution | !l_image_features->empty();

        // ------- Use euclidean distance to find correspondences
        // ------- Grid map is used to limit the search space
        for (const auto& feature : *l_image_features)
        {
          float dist = input.pos_.distance(feature.pos_);
          if (dist < sdist)
          {
            sdist = dist;
            nearest = feature;
          }
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
  int l_i, l_j;
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
            std::cout << "MapLayer::findNearest(Corner) --- " << msg;
            return false;
          }

          // Set cell indexes where to find correspondences
          l_i = i;
          l_j = j;
          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // End search if we found a correspondence in the source cell
          move = DONE;
          break;
        case RIGHT:
          // Compute cell indexes
          l_i = i - level + it;
          l_j = j + level;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            move = DOWN;
            it = 1;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Found solution if there is any feature in the target cell
          if (l_i == i + level)
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
          l_i = i + level;
          l_j = j + level - it;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            move = LEFT;
            it = 1;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (l_j == j - level)
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
          l_i = i + level - it;
          l_j = j - level;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            move = UP;
            it = 1;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (l_i == i - level)
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
          l_i = i - level;
          l_j = j - level + it;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            it = 0;
            move = DONE;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          // The '-1' is to not repeat the first iterator (started on RIGHT)
          if (l_j == j + level - 1)
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

      std::vector<Corner>* l_corners = (*this)(l_i, l_j).corner_features_;
      if (l_corners == nullptr)
      {
      }
      else
      {
        // Found solution if there is any feature in the target cell
        found_solution = found_solution | !l_corners->empty();

        // ------- Use euclidean distance to find correspondences
        // ------- Grid map is used to limit the search space
        for (const auto& feature : *l_corners)
        {
          float dist = input.pos_.distance(feature.pos_);
          if (dist < sdist)
          {
            sdist = dist;
            nearest = feature;
          }
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
  int l_i, l_j;
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
            std::cout << "MapLayer::findNearest(Planar) --- " << msg;
            return false;
          }

          // Set cell indexes where to find correspondences
          l_i = i;
          l_j = j;
          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // End search if we found a correspondence in the source cell
          move = DONE;
          break;
        case RIGHT:
          // Compute cell indexes
          l_i = i - level + it;
          l_j = j + level;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            move = DOWN;
            it = 1;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Found solution if there is any feature in the target cell
          if (l_i == i + level)
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
          l_i = i + level;
          l_j = j + level - it;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            move = LEFT;
            it = 1;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (l_j == j - level)
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
          l_i = i + level - it;
          l_j = j - level;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            move = UP;
            it = 1;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          if (l_i == i - level)
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
          l_i = i - level;
          l_j = j - level + it;
          try
          {
            check(l_i, l_j);
          }
          catch (char const* msg)
          {
            it = 0;
            move = DONE;
            continue;
          }

          // The iteration is valid since (l_i, l_j) passed in the try - catch
          valid_iteration = true;
          // Update the next movement and the iterator
          // The '-1' is to not repeat the first iterator (started on RIGHT)
          if (l_j == j + level - 1)
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

      std::vector<Planar>* l_planars = (*this)(l_i, l_j).planar_features_;
      if (l_planars == nullptr)
      {
      }
      else
      {
        // Found solution if there is any feature in the target cell
        found_solution = found_solution | !l_planars->empty();

        // ------- Use euclidean distance to find correspondences
        // ------- Grid map is used to limit the search space
        for (const auto& feature : *l_planars)
        {
          float dist = input.pos_.distance(feature.pos_);
          if (dist < sdist)
          {
            sdist = dist;
            nearest = feature;
          }
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

  std::vector<ImageFeature>* l_image_features = (*this)(i, j).surf_features_;
  if (l_image_features == nullptr)
  {
    return false;
  }
  else
  {
    for (const auto& feature : *l_image_features)
    {
      dist = input.pos_.distance(feature.pos_);
      if (dist < min_dist)
      {
        min_dist = dist;
        nearest = feature;
      }
    }
  }

  return true;
}

OccupancyMap::OccupancyMap(const Parameters& params, const Pose& origin_offset)
{
  // Read input parameters
  origin_.x_ = params.gridmap_origin_x_ + origin_offset.x_;
  origin_.y_ = params.gridmap_origin_y_ + origin_offset.y_;
  origin_.z_ = params.gridmap_origin_z_;
  resolution_ = params.gridmap_resolution_;
  resolution_z_ = resolution_ / 4;
  width_ = params.gridmap_width_;
  lenght_ = params.gridmap_lenght_;
  height_ = params.gridmap_height_;
  zmin_ = 0;
  zmax_ = static_cast<int>(std::round(height_ / resolution_z_)) - 1;
  planes_ = {};

  // Initialize multi-layer grid map
  float i = origin_.z_;
  while (i < origin_.z_ + height_)
  {
    int layer_num;
    getLayerNumber(i, layer_num);
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
  this->planes_ = grid_map.planes_;
}

bool OccupancyMap::getLayerNumber(const float& z, int& layer_num) const
{
  layer_num = static_cast<int>(std::round((z - origin_.z_) / resolution_z_));
  return !(layer_num < zmin_ || layer_num > zmax_);
}

bool OccupancyMap::insert(const SemanticFeature& l_landmark, const int& id)
{
  int layer_num;
  getLayerNumber(0, layer_num);
  return layers_map_[layer_num].insert(l_landmark, id);
}

bool OccupancyMap::insert(const ImageFeature& l_feature)
{
  int layer_num;
  if (getLayerNumber(l_feature.pos_.z_, layer_num))
  {
    return layers_map_[layer_num].insert(l_feature);
  }
  else
  {
    return false;
  }
}

bool OccupancyMap::insert(const Corner& l_feature)
{
  int layer_num;
  if (getLayerNumber(l_feature.pos_.z_, layer_num))
  {
    return layers_map_[layer_num].insert(l_feature);
  }
  else
  {
    return false;
  }
}

bool OccupancyMap::insert(const Planar& l_feature)
{
  int layer_num;
  if (getLayerNumber(l_feature.pos_.z_, layer_num))
  {
    return layers_map_[layer_num].insert(l_feature);
  }
  else
  {
    return false;
  }
}

bool OccupancyMap::update(const SemanticFeature& new_landmark, const int& id, const float& i, const float& j)
{
  int layer_num;
  getLayerNumber(0, layer_num);
  return layers_map_[layer_num].update(new_landmark, id, i, j);
}

bool OccupancyMap::update(const Corner& old_corner, const Corner& new_corner)
{
  int old_layer_num;
  int new_layer_num;
  getLayerNumber(old_corner.pos_.z_, old_layer_num);
  getLayerNumber(new_corner.pos_.z_, new_layer_num);

  if (old_layer_num == new_layer_num)
  {
    return layers_map_[old_layer_num].update(old_corner, new_corner);
  }
  else
  {
    // Compute grid coordinates for the floating point old corner location
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = static_cast<int>(std::round(old_corner.pos_.x_ / resolution_ + .49));
    int l_j = static_cast<int>(std::round(old_corner.pos_.y_ / resolution_ + .49));

    // Access cell of old corner
    Cell l_cell = layers_map_[old_layer_num](l_i, l_j);
    // Get all the corner in the given cell
    std::vector<Corner>* l_corners = l_cell.corner_features_;

    if (l_corners == nullptr)
    {
    }
    else
    {
      // Find the corner and update it
      for (size_t i = 0; i < l_corners->size(); i++)
      {
        Corner l_corner = (*l_corners)[i];

        if (l_corner.pos_.x_ == old_corner.pos_.x_ && l_corner.pos_.y_ == old_corner.pos_.y_ &&
            l_corner.pos_.z_ == old_corner.pos_.z_)
        {
          layers_map_[old_layer_num](l_i, l_j).corner_features_->erase(
              layers_map_[old_layer_num](l_i, l_j).corner_features_->begin() + i);

          insert(new_corner);

          return true;
        }
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
  int old_layer_num;
  int new_layer_num;
  if (!getLayerNumber(old_planar.pos_.z_, old_layer_num))
  {
    return false;
  }
  if (!getLayerNumber(new_planar.pos_.z_, new_layer_num))
  {
    return false;
  }

  if (old_layer_num == new_layer_num)
  {
    return layers_map_[old_layer_num].update(old_planar, new_planar);
  }
  else
  {
    // Compute grid coordinates for the floating point old planar location
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = static_cast<int>(std::round(old_planar.pos_.x_ / resolution_ + .49));
    int l_j = static_cast<int>(std::round(old_planar.pos_.y_ / resolution_ + .49));

    // Access cell of old planar
    Cell l_cell = layers_map_[old_layer_num](l_i, l_j);
    // Get all the planar in the given cell
    std::vector<Planar>* l_planars = l_cell.planar_features_;

    if (l_planars == nullptr)
    {
    }
    else
    {
      // Find the planar and update it
      for (size_t i = 0; i < l_planars->size(); i++)
      {
        Planar l_planar = (*l_planars)[i];

        if (l_planar.pos_.x_ == old_planar.pos_.x_ && l_planar.pos_.y_ == old_planar.pos_.y_ &&
            l_planar.pos_.z_ == old_planar.pos_.z_)
        {
          layers_map_[old_layer_num](l_i, l_j).planar_features_->erase(
              layers_map_[old_layer_num](l_i, l_j).planar_features_->begin() + i);

          insert(new_planar);

          return true;
        }
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
  int old_layer_num;
  int new_layer_num;
  if (!getLayerNumber(old_image_feature.pos_.z_, old_layer_num))
  {
    return false;
  }
  if (!getLayerNumber(new_image_feature.pos_.z_, new_layer_num))
  {
    return false;
  }

  if (old_layer_num == new_layer_num)
  {
    return layers_map_[old_layer_num].update(old_image_feature, new_image_feature);
  }
  else
  {
    // Compute grid coordinates for the floating point old corner location
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = static_cast<int>(std::round(old_image_feature.pos_.x_ / resolution_ + .49));
    int l_j = static_cast<int>(std::round(old_image_feature.pos_.y_ / resolution_ + .49));

    // Access cell of old corner
    Cell l_cell = layers_map_[old_layer_num](l_i, l_j);
    // Get all the corner in the given cell
    std::vector<ImageFeature>* l_image_features = l_cell.surf_features_;

    if (l_image_features == nullptr)
    {
    }
    else
    {
      // Find the corner and update it
      for (size_t i = 0; i < l_image_features->size(); i++)
      {
        ImageFeature l_image_feature = (*l_image_features)[i];

        if (l_image_feature.pos_.x_ == old_image_feature.pos_.x_ &&
            l_image_feature.pos_.y_ == old_image_feature.pos_.y_ &&
            l_image_feature.pos_.z_ == old_image_feature.pos_.z_)
        {
          layers_map_[old_layer_num](l_i, l_j).surf_features_->erase(
              layers_map_[old_layer_num](l_i, l_j).surf_features_->begin() + i);

          insert(new_image_feature);

          return true;
        }
      }
    }
  }

  std::cout << "WARNING (OcuppancyMap::update): Trying to update an image feature that is "
               "not on the map... "
            << std::endl;
  return false;
}

void OccupancyMap::downsampleCorners()
{
  for (auto& mlayer : layers_map_)
    mlayer.second.downsampleCorners();
}

void OccupancyMap::downsamplePlanars()
{
  for (auto& mlayer : layers_map_)
  {
    mlayer.second.downsamplePlanars();
  }
}

bool OccupancyMap::getAdjacent(const float& x, const float& y, const float& z, const int& layers,
                               std::vector<Cell>& adjacent)
{
  // Set up data needed to compute the routine
  std::vector<Cell> up_cells;
  std::vector<Cell> layer_cells;
  std::vector<Cell> down_cells;

  int layer_num;
  if (!getLayerNumber(z, layer_num))
  {
    return false;
  }

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

  int layer_num;
  if (!getLayerNumber(input.pos_.z_, layer_num))
  {
    return false;
  }

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

  int layer_num;
  if (!getLayerNumber(input.pos_.z_, layer_num))
  {
    return false;
  }

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

  int layer_num;
  if (!getLayerNumber(input.pos_.z_, layer_num))
  {
    return false;
  }

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
  int layer_num;
  if (!getLayerNumber(input.pos_.z_, layer_num))
  {
    return false;
  }
  else
  {
    return layers_map_[layer_num].findNearestOnCell(input, nearest);
  }
}

bool OccupancyMap::rayTrace(const std::vector<Point>& pts, const Point& sensor_origin)
{
  //  for (const auto& pt : pts)
  //  {
  //    // Check if point lies inside the map
  //    if (!isInside(pt.x_, pt.y_, pt.z_))
  //    {
  //      continue;
  //    }
  //
  //    // Voxel traverse - get grid map points traversed by the ray
  //    std::vector<Point> voxels = voxelTraversal(sensor_origin, pt);
  //
  //    // Delete the traversed occupied cell points
  //    uint32_t num_pts =
  //        (voxels.size() > 10) ? voxels.size() - 10 : 0;  // we do not want to remove the last points of the ray
  //    for (uint32_t i = 0; i < num_pts; i++)
  //    {
  //      Point f_pt(voxels[i].x_ * resolution_, voxels[i].y_ * resolution_, voxels[i].z_ * resolution_z_);
  //      Cell* c = &(*this)(f_pt.x_, f_pt.y_, f_pt.z_);
  //
  //      // Increment the number of rays that have traversed this cell
  //      c->traverses_planars++;
  //
  //      // If the cell is not empty and the number of traverses if higher than the number of hits, we will erase all
  //      the
  //      // planar information from it
  //      if (!c->planar_features_->empty() && (c->traverses_planars >= c->hits_planars))
  //      {
  //        *c->planar_features_ = {};
  //        *c->candidate_planar_features_ = {};
  //        *c->n_candidate_planars_ = 0;
  //      }
  //    }
  //  }

  return true;
}

std::vector<Point> OccupancyMap::voxelTraversal(const Point& ray_start, const Point& ray_end)
{
  std::vector<Point> visited_voxels;

  // This id of the first/current voxel hit by the ray.
  // Using floor (round down) is actually very important,
  // the implicit int-casting will round up for negative numbers.
  Point current_voxel(std::floor(ray_start.x_ / resolution_), std::floor(ray_start.y_ / resolution_),
                      std::floor(ray_start.z_ / resolution_z_));

  // The id of the last voxel hit by the ray.
  // TODO: what happens if the end point is on a border?
  Point last_voxel(std::floor(ray_end.x_ / resolution_), std::floor(ray_end.y_ / resolution_),
                   std::floor(ray_end.z_ / resolution_z_));

  // Compute normalized ray direction.
  Point ray = ray_end - ray_start;
  // ray.normalize();

  // In which direction the voxel ids are incremented.
  float stepX = (ray.x_ >= 0) ? 1 : -1;  // correct
  float stepY = (ray.y_ >= 0) ? 1 : -1;  // correct
  float stepZ = (ray.z_ >= 0) ? 1 : -1;  // correct

  // Distance along the ray to the next voxel border from the current position (tMaxX, tMaxY, tMaxZ).
  float next_voxel_boundary_x = (current_voxel.x_ + stepX) * resolution_;    // correct
  float next_voxel_boundary_y = (current_voxel.y_ + stepY) * resolution_;    // correct
  float next_voxel_boundary_z = (current_voxel.z_ + stepZ) * resolution_z_;  // correct

  // tMaxX, tMaxY, tMaxZ -- distance until next intersection with voxel-border
  // the value of t at which the ray crosses the first vertical voxel boundary
  float tMaxX = (ray.x_ != 0) ? (next_voxel_boundary_x - ray_start.x_) / ray.x_ : std::numeric_limits<float>::max();  //
  float tMaxY = (ray.y_ != 0) ? (next_voxel_boundary_y - ray_start.y_) / ray.y_ : std::numeric_limits<float>::max();  //
  float tMaxZ = (ray.z_ != 0) ? (next_voxel_boundary_z - ray_start.z_) / ray.z_ : std::numeric_limits<float>::max();  //

  // tDeltaX, tDeltaY, tDeltaZ --
  // how far along the ray we must move for the horizontal component to equal the width of a voxel
  // the direction in which we traverse the grid
  // can only be FLT_MAX if we never go in that direction
  float tDeltaX = (ray.x_ != 0) ? resolution_ / ray.x_ * stepX : std::numeric_limits<float>::max();
  float tDeltaY = (ray.y_ != 0) ? resolution_ / ray.y_ * stepY : std::numeric_limits<float>::max();
  float tDeltaZ = (ray.z_ != 0) ? resolution_z_ / ray.z_ * stepZ : std::numeric_limits<float>::max();

  Point diff(0, 0, 0);
  bool neg_ray = false;
  if (current_voxel.x_ != last_voxel.x_ && ray.x_ < 0)
  {
    diff.x_--;
    neg_ray = true;
  }
  if (current_voxel.y_ != last_voxel.y_ && ray.y_ < 0)
  {
    diff.y_--;
    neg_ray = true;
  }
  if (current_voxel.z_ != last_voxel.z_ && ray.z_ < 0)
  {
    diff.z_--;
    neg_ray = true;
  }
  visited_voxels.push_back(current_voxel);
  if (neg_ray)
  {
    current_voxel = current_voxel + diff;
    visited_voxels.push_back(current_voxel);
  }

  while (last_voxel != current_voxel)
  {
    if (tMaxX < tMaxY)
    {
      if (tMaxX < tMaxZ)
      {
        current_voxel.x_ += stepX;
        tMaxX += tDeltaX;
      }
      else
      {
        current_voxel.z_ += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
    else
    {
      if (tMaxY < tMaxZ)
      {
        current_voxel.y_ += stepY;
        tMaxY += tDeltaY;
      }
      else
      {
        current_voxel.z_ += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
    visited_voxels.push_back(current_voxel);
  }

  return visited_voxels;
}

}  // namespace vineslam