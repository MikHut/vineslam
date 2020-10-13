#pragma once

#include <params.hpp>
#include <math/point.hpp>
#include <feature.hpp>

#include <iostream>
#include <iomanip>
#include <vector>
#include <map>

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

  // List of landmarks and features at each cell
  std::map<int, SemanticFeature> landmarks;
  std::vector<ImageFeature>      surf_features;
  std::vector<Corner>            corner_features;

private:
};

class OccupancyMap
{
public:
  // Class constructor
  // - initializes the grid map given the input parameters
  explicit OccupancyMap(const Parameters& params);

  // Copy contructor
  OccupancyMap(const OccupancyMap& grid_map);

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
    return m_gmap[m_i +
                  m_j * static_cast<int>(std::round(width / resolution + .49))];
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
      throw "Exception: Access to grid map out of bounds";
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

  // Since Landmark map is built with a KF, Landmarks position change in each
  // iteration. This routine updates the position of a given Landmark
  bool update(const SemanticFeature& new_landmark,
              const int&             id,
              const float&           i,
              const float&           j);

  // Updates a corner 3D feature location
  bool update(const Corner& old_corner, const Corner& new_corner);

  // Updates a image 3D feature location
  bool update(const ImageFeature& old_image_feature,
              const ImageFeature& new_image_feature);

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

  // Find nearest neighbor of a feature considering adjacent cells
  bool findNearest(const ImageFeature& input,
                   ImageFeature&       nearest,
                   float&              sdist,
                   float&              ddist);
  // Find nearest neighbor of a feature on its cell
  bool findNearestOnCell(const ImageFeature& input, ImageFeature& nearest);

  // Returns true if the map has no features or landmarks
  bool empty() const
  {
    return (n_surf_features == 0 && n_landmarks == 0 && n_corner_features == 0);
  }

  // Delete all features in the map
  void clear()
  {
    for (auto& cell : m_gmap) {
      cell.corner_features.clear();
      cell.surf_features.clear();
      cell.landmarks.clear();
    }

    n_corner_features = 0;
    n_surf_features   = 0;
    n_landmarks       = 0;
  }

  // Number of features and landmarks in the map
  int n_surf_features;
  int n_corner_features;
  int n_landmarks;

  // Grid map dimensions
  // NOTE: map corners are in reference to the given origin
  point origin;
  float width;
  float height;
  float resolution;
  // Search metric to use: euclidean / descriptor
  std::string metric;

private:
  // Private grid map to store all the cells
  std::vector<Cell> m_gmap;
};

} // namespace vineslam
