#pragma once

#include "../feature/semantic.hpp"
#include "../feature/visual.hpp"
#include "../feature/three_dimensional.hpp"
#include "../params.hpp"
#include "../math/Point.hpp"

#include <iostream>
#include <vector>
#include <map>
#include <set>

namespace vineslam
{
class Cell
{
public:
  // Default constructor
  Cell() = default;

  // Inserts a landmark with a given id
  void insert(const int& id, const SemanticFeature& l_landmark)
  {
    landmarks_[id] = l_landmark;
  }

  // Inserts a image feature in the features array
  void insert(const ImageFeature& l_feature)
  {
    surf_features_.push_back(l_feature);
  }

  // Inserts a corner feature in the features array
  void insert(const Corner& l_feature)
  {
    corner_features_.push_back(l_feature);
  }

  // Inserts a planar feature in the features array
  void insert(const Planar& l_feature)
  {
    planar_features_.push_back(l_feature);
  }

  // Inserts a point in the points array
  void insert(const Point& l_point)
  {
    points_.push_back(l_point);
  }

  // List of landmarks, features, and points at each cell
  std::map<int, SemanticFeature> landmarks_;
  std::vector<ImageFeature> surf_features_;
  std::vector<Corner> corner_features_;
  std::vector<Planar> planar_features_;
  std::vector<Point> points_;

  // List of candidate landmarks, features, and points at each cell
  std::vector<Corner> candidate_corner_features_;
  std::vector<Planar> candidate_planar_features_;

  // Number of observations for each type of feature
  uint32_t n_candidate_corners_{0};
  uint32_t n_candidate_planars_{0};

  // Number of hits and ray traverses
  uint32_t hits_planars{0};
  uint32_t traverses_planars{0};

private:
};

class MapLayer
{
public:
  MapLayer() = default;

  // Class constructor
  // - initializes the grid map given the input parameters
  MapLayer(const Parameters& params, const Pose& origin_offset);

  // Copy contructor
  MapLayer(const MapLayer& grid_map);

  // 2D grid map direct access to cell coordinates
  Cell& operator()(int i, int j)
  {
    // Verify validity of indexing
    try
    {
      check(i, j);
    }
    catch (char const* msg)
    {
      std::cout << msg;
      std::cout << "Returning last grid element ..." << std::endl;

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
  Cell& operator()(float i, float j)
  {
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = static_cast<int>(std::round(i / resolution_ + .49));
    int l_j = static_cast<int>(std::round(j / resolution_ + .49));

    return (*this)(l_i, l_j);
  }

  // Check out of bounds indexing
  void check(const int& i, const int& j)
  {
    // Compute indexes having into account that grid map considers negative values
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
    int l_j = j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
    int index = l_i + (l_j * static_cast<int>(std::round(width_ / resolution_ + .49)));

    // Trough exception if out of bounds indexing
    if (index >= cell_vec_.size() - 1 || index < 0)
      throw "Exception: Access to grid map out of bounds\n";
  }

  // Check if a point if inside the map
  bool isInside(const float& i, const float& j)
  {
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int l_i = static_cast<int>(std::round(i / resolution_ + .49));
    int l_j = static_cast<int>(std::round(j / resolution_ + .49));

    // Compute indexes having into account that grid map considers negative values
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int ll_i = l_i - static_cast<int>(std::round(origin_.x_ / resolution_ + .49));
    int ll_j = l_j - static_cast<int>(std::round(origin_.y_ / resolution_ + .49));
    int index = ll_i + (ll_j * static_cast<int>(std::round(width_ / resolution_ + .49)));

    if (index >= cell_vec_.size() - 1 || index < 0)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  // Define iterator to provide access to the cells array
  typedef std::vector<Cell>::iterator iterator;
  // Return members to provide access to the cells array
  iterator begin()
  {
    return cell_vec_.begin();
  }
  iterator end()
  {
    return cell_vec_.end();
  }

  // Insert a Landmark using the direct grid coordinates
  bool insert(const SemanticFeature& l_landmark, const int& id, const int& i, const int& j);

  // Insert a Landmark given a Feature/Landmark location
  bool insert(const SemanticFeature& l_landmark, const int& id);

  // Insert a Image Feature using the direct grid coordinates
  bool insert(const ImageFeature& l_feature, const int& i, const int& j);
  // Insert a Image Feature given a Feature/Landmark location
  bool insert(const ImageFeature& l_feature);

  // Insert a Image Feature using the direct grid coordinates
  bool insert(const Corner& l_feature, const int& i, const int& j);
  // Insert a Image Feature given a Feature/Landmark location
  bool insert(const Corner& l_feature);

  // Insert a Image Feature using the direct grid coordinates
  bool insert(const Planar& l_feature, const int& i, const int& j);
  // Insert a Image Feature given a Feature/Landmark location
  bool insert(const Planar& l_feature);

  // Insert a point using the direct grid coordinates
  bool insert(const Point& l_point, const int& i, const int& j);
  // Insert a point given a its location
  bool insert(const Point& l_point);

  // Since Landmark map is built with a KF, Landmarks position change in each
  // iteration. This routine updates the position of a given Landmark
  bool update(const SemanticFeature& new_landmark, const int& id, const float& i, const float& j);

  // Updates a corner 3D feature location
  bool update(const Corner& old_corner, const Corner& new_corner);

  // Updates a planar 3D feature location
  bool update(const Planar& old_planar, const Planar& new_planar);

  // Updates a image 3D feature location
  bool update(const ImageFeature& old_image_feature, const ImageFeature& new_image_feature);

  // Downsamples the corner map
  void downsampleCorners();

  // Downsamples the planar map
  void downsamplePlanars();

  // Method to get all the adjacent cells to a given cell
  bool getAdjacent(const int& i, const int& j, const int& layers, std::vector<Cell>& adjacent);
  // Method to get all the adjacent cells to a given cell given a Feature/Landmark
  // location
  bool getAdjacent(const float& i, const float& j, const int& layers, std::vector<Cell>& adjacent);

  // Find nearest neighbor of an image feature considering adjacent cells
  bool findNearest(const ImageFeature& input, ImageFeature& nearest, float& ddist);

  // Find nearest neighbor of a corner feature considering adjacent cells
  bool findNearest(const Corner& input, Corner& nearest, float& sdist);

  // Find nearest neighbor of a planar feature considering adjacent cells
  bool findNearest(const Planar& input, Planar& nearest, float& sdist);

  // Find nearest neighbor of a feature on its cell
  bool findNearestOnCell(const ImageFeature& input, ImageFeature& nearest);

  // Getter functions
  std::vector<Corner> getCorners() const
  {
    std::vector<Corner> out_corners;
    for (const auto& i : corner_set_)
      for (const auto& corner : cell_vec_[i].corner_features_)
        out_corners.push_back(corner);

    return out_corners;
  }
  std::vector<Planar> getPlanars() const
  {
    std::vector<Planar> out_planars;
    for (const auto& i : planar_set_)
      for (const auto& planar : cell_vec_[i].planar_features_)
        out_planars.push_back(planar);

    return out_planars;
  }
  std::vector<ImageFeature> getImageFeatures() const
  {
    std::vector<ImageFeature> out_surf_features;
    for (const auto& i : surf_set_)
      for (const auto& img_feature : cell_vec_[i].surf_features_)
        out_surf_features.push_back(img_feature);

    return out_surf_features;
  }

  // Returns true if the map has no features or landmarks
  bool empty() const
  {
    return (n_surf_features_ == 0 && n_landmarks_ == 0 && n_corner_features_ == 0 && n_planar_features_ == 0 &&
            n_points_ == 0);
  }

  // Delete all features in the map
  void clear()
  {
    // ************************ WARNING ********************** //
    // ************** This function is very slow ************* //
    // ******************************************************* //
    for (auto& cell : cell_vec_)
    {
      cell.corner_features_.shrink_to_fit();
      cell.planar_features_.shrink_to_fit();
      cell.surf_features_.shrink_to_fit();
      cell.landmarks_.clear();
      cell.points_.shrink_to_fit();
    }

    n_corner_features_ = 0;
    n_planar_features_ = 0;
    n_surf_features_ = 0;
    n_landmarks_ = 0;
    n_points_ = 0;
  }

  // Number of features, landmarks, and points in the map
  int n_surf_features_{};
  int n_corner_features_{};
  int n_planar_features_{};
  int n_landmarks_{};
  int n_points_{};

  // Minimum number of observations to add a corners or planar feature to the map
  uint32_t min_planar_obsvs_;
  uint32_t min_corner_obsvs_;

  // Grid map dimensions
  Point origin_;
  float resolution_;
  float width_;
  float lenght_;

private:
  // Private grid map to store all the cells
  std::vector<Cell> cell_vec_;

  // Pointer arrays to occupied cells with each feature class
  std::set<int> surf_set_;
  std::set<int> corner_set_;
  std::set<int> planar_set_;
  std::set<int> landmark_set_;
  std::set<int> point_set_;
};

class OccupancyMap
{
public:
  // Class constructor
  // - initializes the multi-layer grid map given the input parameters
  OccupancyMap(const Parameters& params, const Pose& origin_offset);

  // Copy constructor
  OccupancyMap(const OccupancyMap& grid_map);

  // Access map layer
  MapLayer& operator()(float z)
  {
    int layer_num;
    getLayerNumber(z, layer_num);
    layer_num = (layer_num < zmin_) ? zmin_ : layer_num;
    layer_num = (layer_num > zmax_) ? zmax_ : layer_num;

    return layers_map_[layer_num];
  }

  // 3D grid map access to cell coordinates
  Cell& operator()(float x, float y, float z)
  {
    int layer_num;
    getLayerNumber(z, layer_num);
    layer_num = (layer_num < zmin_) ? zmin_ : layer_num;
    layer_num = (layer_num > zmax_) ? zmax_ : layer_num;

    return layers_map_[layer_num](x, y);
  }

  // Check if a point is inside the map
  bool isInside(const float& x, const float& y, const float& z)
  {
    int layer_num;
    getLayerNumber(z, layer_num);
    if (layer_num > zmax_ || layer_num < zmin_)
    {
      return false;
    }
    else
    {
      if (!layers_map_[layer_num].isInside(x, y))
      {
        return false;
      }
      else
      {
        return true;
      }
    }
  }

  // Define iterator to provide access to the layers array
  typedef std::map<int, MapLayer>::iterator iterator;
  // Return members to provide access to the cells array
  iterator begin()
  {
    return layers_map_.begin();
  }
  iterator end()
  {
    return layers_map_.end();
  }

  // Insert a Landmark given a Feature/Landmark location
  bool insert(const SemanticFeature& l_landmark, const int& id);

  // Insert a Image Feature given a Feature/Landmark location
  bool insert(const ImageFeature& l_feature);

  // Insert a corner given a Feature/Landmark location
  bool insert(const Corner& l_feature);

  // Insert a planar feature given a Feature/Landmark location
  bool insert(const Planar& l_feature);

  // Insert a point given a its location
  bool insert(const Point& l_point);

  // Downsamples the corner map
  void downsampleCorners();

  // Downsamples the planar map
  void downsamplePlanars();

  // Since Landmark map is built with a KF, Landmarks position change in each
  // iteration. This routine updates the position of a given Landmark
  bool update(const SemanticFeature& new_landmark, const int& id, const float& i, const float& j);

  // Updates a corner 3D feature location
  bool update(const Corner& old_corner, const Corner& new_corner);

  // Updates a planar 3D feature location
  bool update(const Planar& old_planar, const Planar& new_planar);

  // Updates a image 3D feature location
  bool update(const ImageFeature& old_image_feature, const ImageFeature& new_image_feature);

  // Method to get all the adjacent cells to a given cell given a Feature/Landmark
  // location
  bool getAdjacent(const float& x, const float& y, const float& z, const int& layers, std::vector<Cell>& adjacent);

  // Find nearest neighbor of a feature considering adjacent cells
  bool findNearest(const ImageFeature& input, ImageFeature& nearest, float& ddist);

  // Find nearest neighbor of a corner feature considering adjacent cells
  bool findNearest(const Corner& input, Corner& nearest, float& sdist);

  // Find nearest neighbor of a planar feature considering adjacent cells
  bool findNearest(const Planar& input, Planar& nearest, float& sdist);

  // Find nearest neighbor of a feature on its cell
  bool findNearestOnCell(const ImageFeature& input, ImageFeature& nearest);
  // Recover the layer number from the feature z component
  bool getLayerNumber(const float& z, int& layer_num) const;
  // Returns all the grid map cells transversed by a ray
  std::vector<Point> voxelTraversal(const Point& ray_start, const Point& ray_end);
  // Performs ray trace for a specific set of points
  bool rayTrace(const std::vector<Point>& pts, const Point& sensor_origin);

  // Getter functions
  std::vector<Corner> getCorners()
  {
    std::vector<Corner> out_corners;
    for (const auto& layer : layers_map_)
    {
      std::vector<Corner> l_corners = layer.second.getCorners();
      out_corners.insert(out_corners.end(), l_corners.begin(), l_corners.end());
    }

    return out_corners;
  }
  std::vector<Planar> getPlanars()
  {
    std::vector<Planar> out_planars;
    for (const auto& layer : layers_map_)
    {
      std::vector<Planar> l_planars = layer.second.getPlanars();
      out_planars.insert(out_planars.end(), l_planars.begin(), l_planars.end());
    }

    return out_planars;
  }
  std::vector<ImageFeature> getImageFeatures()
  {
    std::vector<ImageFeature> out_surf_features;
    for (const auto& layer : layers_map_)
    {
      std::vector<ImageFeature> l_surf_features = layer.second.getImageFeatures();
      out_surf_features.insert(out_surf_features.end(), l_surf_features.begin(), l_surf_features.end());
    }

    return out_surf_features;
  }

  // Returns true is none layer has features/landmarks
  bool empty() const
  {
    bool is_empty = false;
    for (const auto& layer : layers_map_)
      is_empty |= layer.second.empty();

    return is_empty;
  }

  // Delete all layers in the map
  void clear()
  {
    for (auto& layer : layers_map_)
      layer.second.clear();
  }

  // Grid map dimensions
  Point origin_;
  float width_;
  float lenght_;
  float height_;
  float resolution_;
  float resolution_z_;
  int zmin_;
  int zmax_;

  // Global planes handler
  std::vector<SemiPlane> planes_;

private:
  // Private grid map to store all the individual layers
  // (int, MapLayer): (layer number, layer class)
  std::map<int, MapLayer> layers_map_;
};

}  // namespace vineslam
