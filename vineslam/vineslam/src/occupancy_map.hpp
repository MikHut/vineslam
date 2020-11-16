#pragma once

#include <params.hpp>
#include <math/point.hpp>
#include <feature.hpp>

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
  void insert(const int& id, const SemanticFeature& m_landmark)
  {
    landmarks[id] = m_landmark;
  }

  // Inserts a image feature in the features array
  void insert(const ImageFeature& m_feature) { surf_features.push_back(m_feature); }

  // Inserts a corner feature in the features array
  void insert(const Corner& m_feature) { corner_features.push_back(m_feature); }

  // Inserts a planar feature in the features array
  void insert(const Planar& m_feature) { planar_features.push_back(m_feature); }

  // Inserts a point in the points array
  void insert(const point& m_point) { points.push_back(m_point); }

  // List of landmarks, features, and points at each cell
  std::map<int, SemanticFeature> landmarks;
  std::vector<ImageFeature>      surf_features;
  std::vector<Corner>            corner_features;
  std::vector<Planar>            planar_features;
  std::vector<point>             points;

private:
};

class MapLayer
{
public:
  MapLayer() = default;

  // Class constructor
  // - initializes the grid map given the input parameters
  MapLayer(const Parameters& params, const pose& origin_offset);

  // Copy contructor
  MapLayer(const MapLayer& grid_map);

  // 2D grid map direct access to cell coordinates
  Cell& operator()(int i, int j)
  {
    // Verify validity of indexing
    try {
      check(i, j);
    } catch (char const* msg) {
      std::cout << msg;
      std::cout << "Returning last grid element ..." << std::endl;

      return m_gmap[m_gmap.size() - 1];
    }

    // Compute indexes having into account that grid map considers negative values
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int m_i = i - static_cast<int>(std::round(origin.x / resolution + .49));
    int m_j = j - static_cast<int>(std::round(origin.y / resolution + .49));
    int idx = m_i + m_j * static_cast<int>(std::round(width / resolution + .49));

    return m_gmap[idx];
  }

  // 2D grid map access given a Feature/Landmark location
  Cell& operator()(float i, float j)
  {
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int m_i = static_cast<int>(std::round(i / resolution + .49));
    int m_j = static_cast<int>(std::round(j / resolution + .49));

    return (*this)(m_i, m_j);
  }

  // Check out of bounds indexing
  void check(const int& i, const int& j)
  {
    // Compute indexes having into account that grid map considers negative values
    // .49 is to prevent bad approximations (e.g. 1.49 = 1 & 1.51 = 2)
    int m_i   = i - static_cast<int>(std::round(origin.x / resolution + .49));
    int m_j   = j - static_cast<int>(std::round(origin.y / resolution + .49));
    int index = m_i + (m_j * static_cast<int>(std::round(width / resolution + .49)));

    // Trough exception if out of bounds indexing
    if (index >= m_gmap.size() - 1 || index < 0)
      throw "Exception: Access to grid map out of bounds\n";
  }

  // Define iterator to provide access to the cells array
  typedef std::vector<Cell>::iterator iterator;
  // Return members to provide access to the cells array
  iterator begin() { return m_gmap.begin(); }
  iterator end() { return m_gmap.end(); }

  // Insert a Landmark using the direct grid coordinates
  bool insert(const SemanticFeature& m_landmark,
              const int&             id,
              const int&             i,
              const int&             j);

  // Insert a Landmark given a Feature/Landmark location
  bool insert(const SemanticFeature& m_landmark, const int& id);

  // Insert a Image Feature using the direct grid coordinates
  bool insert(const ImageFeature& m_feature, const int& i, const int& j);
  // Insert a Image Feature given a Feature/Landmark location
  bool insert(const ImageFeature& m_feature);

  // Insert a Image Feature using the direct grid coordinates
  bool insert(const Corner& m_feature, const int& i, const int& j);
  // Insert a Image Feature given a Feature/Landmark location
  bool insert(const Corner& m_feature);

  // Insert a Image Feature using the direct grid coordinates
  bool insert(const Planar& m_feature, const int& i, const int& j);
  // Insert a Image Feature given a Feature/Landmark location
  bool insert(const Planar& m_feature);

  // Insert a point using the direct grid coordinates
  bool insert(const point& m_point, const int& i, const int& j);
  // Insert a point given a its location
  bool insert(const point& m_point);

  // Since Landmark map is built with a KF, Landmarks position change in each
  // iteration. This routine updates the position of a given Landmark
  bool update(const SemanticFeature& new_landmark,
              const int&             id,
              const float&           i,
              const float&           j);

  // Updates a corner 3D feature location
  bool update(const Corner& old_corner, const Corner& new_corner);

  // Updates a planar 3D feature location
  bool update(const Planar& old_planar, const Planar& new_planar);

  // Updates a image 3D feature location
  bool update(const ImageFeature& old_image_feature,
              const ImageFeature& new_image_feature);

  // Downsamples the corner map
  void downsampleCorners();

  // Downsamples the planar map
  void downsamplePlanars();

  // Method to get all the adjacent cells to a given cell
  bool getAdjacent(const int&         i,
                   const int&         j,
                   const int&         layers,
                   std::vector<Cell>& adjacent);
  // Method to get all the adjacent cells to a given cell given a Feature/Landmark
  // location
  bool getAdjacent(const float&       i,
                   const float&       j,
                   const int&         layers,
                   std::vector<Cell>& adjacent);

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
    for (const auto& i : corner_indexes)
      for (const auto& corner : m_gmap[i].corner_features)
        out_corners.push_back(corner);

    return out_corners;
  }
  std::vector<Planar> getPlanars() const
  {
    std::vector<Planar> out_planars;
    for (const auto& i : planar_indexes)
      for (const auto& planar : m_gmap[i].planar_features)
        out_planars.push_back(planar);

    return out_planars;
  }

  // Returns true if the map has no features or landmarks
  bool empty() const
  {
    return (n_surf_features == 0 && n_landmarks == 0 && n_corner_features == 0 &&
            n_planar_features == 0 && n_points == 0);
  }

  // Delete all features in the map
  void clear()
  {
    for (auto& cell : m_gmap) {
      cell.corner_features.clear();
      cell.planar_features.clear();
      cell.surf_features.clear();
      cell.landmarks.clear();
      cell.points.clear();
    }

    n_corner_features = 0;
    n_planar_features = 0;
    n_surf_features   = 0;
    n_landmarks       = 0;
    n_points          = 0;
  }

  // Number of features, landmarks, and points in the map
  int n_surf_features{};
  int n_corner_features{};
  int n_planar_features{};
  int n_landmarks{};
  int n_points{};

  // Grid map dimensions
  // NOTE: map corners are in reference to the given origin
  point origin;
  float width{};
  float lenght{};
  float resolution{};

private:
  // Private grid map to store all the cells
  std::vector<Cell>  m_gmap;
  std::map<int, int> m_imap;

  // Pointer arrays to occupied cells with each feature class
  std::set<int> surf_indexes;
  std::set<int> corner_indexes;
  std::set<int> planar_indexes;
  std::set<int> landmark_indexes;
  std::set<int> point_indexes;
};

class OccupancyMap
{
public:
  // Class constructor
  // - initializes the multi-layer grid map given the input parameters
  OccupancyMap(const Parameters& params, const pose& origin_offset);

  // Copy constructor
  OccupancyMap(const OccupancyMap& grid_map);

  // Access map layer
  MapLayer& operator()(float z)
  {
    int layer_num = getLayerNumber(z);
    layer_num     = (layer_num < zmin) ? zmin : layer_num;
    layer_num     = (layer_num > zmax) ? zmax : layer_num;

    return m_layers[layer_num];
  }

  // 3D grid map access to cell coordinates
  Cell& operator()(float x, float y, float z)
  {
    int layer_num = getLayerNumber(z);
    layer_num     = (layer_num < zmin) ? zmin : layer_num;
    layer_num     = (layer_num > zmax) ? zmax : layer_num;

    return m_layers[layer_num](x, y);
  }

  // Define iterator to provide access to the layers array
  typedef std::map<int, MapLayer>::iterator iterator;
  // Return members to provide access to the cells array
  iterator begin() { return m_layers.begin(); }
  iterator end() { return m_layers.end(); }

  // Insert a Landmark given a Feature/Landmark location
  bool insert(const SemanticFeature& m_landmark, const int& id);

  // Insert a Image Feature given a Feature/Landmark location
  bool insert(const ImageFeature& m_feature);

  // Insert a corner given a Feature/Landmark location
  bool insert(const Corner& m_feature);

  // Insert a planar feature given a Feature/Landmark location
  bool insert(const Planar& m_feature);

  // Insert a point given a its location
  bool insert(const point& m_point);

  // Downsamples the corner map
  void downsampleCorners();

  // Downsamples the planar map
  void downsamplePlanars();

  // Since Landmark map is built with a KF, Landmarks position change in each
  // iteration. This routine updates the position of a given Landmark
  bool update(const SemanticFeature& new_landmark,
              const int&             id,
              const float&           i,
              const float&           j);

  // Updates a corner 3D feature location
  bool update(const Corner& old_corner, const Corner& new_corner);

  // Updates a planar 3D feature location
  bool update(const Planar& old_planar, const Planar& new_planar);

  // Updates a image 3D feature location
  bool update(const ImageFeature& old_image_feature,
              const ImageFeature& new_image_feature);

  // Method to get all the adjacent cells to a given cell given a Feature/Landmark
  // location
  bool getAdjacent(const float&       x,
                   const float&       y,
                   const float&       z,
                   const int&         layers,
                   std::vector<Cell>& adjacent);

  // Find nearest neighbor of a feature considering adjacent cells
  bool findNearest(const ImageFeature& input, ImageFeature& nearest, float& ddist);

  // Find nearest neighbor of a corner feature considering adjacent cells
  bool findNearest(const Corner& input, Corner& nearest, float& sdist);

  // Find nearest neighbor of a planar feature considering adjacent cells
  bool findNearest(const Planar& input, Planar& nearest, float& sdist);

  // Find nearest neighbor of a feature on its cell
  bool findNearestOnCell(const ImageFeature& input, ImageFeature& nearest);
  // Recover the layer number from the feature z component
  int getLayerNumber(const float& z) const;

  // Getter functions
  std::vector<Corner> getCorners()
  {
    std::vector<Corner> out_corners;
    for (const auto& layer : m_layers) {
      std::vector<Corner> m_corners = layer.second.getCorners();
      out_corners.insert(out_corners.end(), m_corners.begin(), m_corners.end());
    }

    return out_corners;
  }
  std::vector<Planar> getPlanars()
  {
    std::vector<Planar> out_planars;
    for (const auto& layer : m_layers) {
      std::vector<Planar> m_planars = layer.second.getPlanars();
      out_planars.insert(out_planars.end(), m_planars.begin(), m_planars.end());
    }

    return out_planars;
  }

  // Returns true is none layer has features/landmarks
  bool empty() const
  {
    bool is_empty = false;
    for (const auto& layer : m_layers) is_empty |= layer.second.empty();

    return is_empty;
  }

  // Delete all layers in the map
  void clear()
  {
    for (auto& layer : m_layers) layer.second.clear();
  }

  // Grid map dimensions
  point origin;
  float width;
  float lenght;
  float height;
  float resolution;
  float resolution_z;
  int   zmin;
  int   zmax;

  // Grid map high level features (not stored in cells)
  std::vector<Plane> map_planes;

private:
  // Private grid map to store all the individual layers
  // (int, MapLayer): (layer number, layer class)
  std::map<int, MapLayer> m_layers;
};

} // namespace vineslam
