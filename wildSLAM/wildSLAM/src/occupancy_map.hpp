#pragma once

#include <math/point.hpp>
#include <landmark.hpp>
#include <feature.hpp>

#include <iostream>
#include <iomanip>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace wildSLAM
{

class Cell
{
public:
  // Default constructor
  Cell() {}

  // Inserts a landmark with a given id
  void insert(const int& id, const Landmark& m_landmark)
  {
    landmarks[id] = m_landmark;
  }

  // Inserts a feature in the features array
  void insert(const Feature& m_feature) { features.push_back(m_feature); }

  // List of landmarks and features at each cell
  std::map<int, Landmark> landmarks;
  std::vector<Feature>    features;

private:
};

class OccupancyMap
{
public:
  // Class constructor
  // - initializes the grid map given the input parameters
  OccupancyMap(const std::string& config_path);

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
      throw "Exception: Access to grid map out of bounds\n";
  }

  // Define iterator to provide access to the cells array
  typedef std::vector<Cell>::iterator iterator;
  // Return members to provide access to the cells array
  iterator begin() { return m_gmap.begin(); }
  iterator end() { return m_gmap.end(); }

  // Insert a Landmark using the direct grid coordinates
  bool insert(const Landmark& m_landmark, const int& id, const int& i, const int& j);
  // Insert a Landmark given a Feature/Landmark location
  bool
  insert(const Landmark& m_landmark, const int& id, const float& i, const float& j);

  // Insert a Feature using the direct grid coordinates
  bool insert(const Feature& m_feature, const int& i, const int& j);
  // Insert a Feature given a Feature/Landmark location
  bool insert(const Feature& m_feature, const float& i, const float& j);

  // Since Landmark map is built with a KF, Landmarks position change in each
  // iteration. This routine updates the position of a given Landmark
  bool update(const Landmark& new_landmark,
              const int&      id,
              const float&    i,
              const float&    j);

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
  bool findNearest(const Feature& input, Feature& nearest);
  // Find nearest neighbor of a feature on its cell
  bool findNearestOnCell(const Feature& input, Feature& nearest);

  // Returns true if the map has no features or landmarks
  bool empty() { return (n_features == 0 && n_landmarks == 0); }

private:
  // Private grid map to store all the cells
  std::vector<Cell> m_gmap;

  // Number of features and landmarks in the map
  int n_features;
  int n_landmarks;

  // Grid map dimensions
  // NOTE: corners are in reference to the given origin
  point origin;
  float width;
  float height;
  float resolution;
  // Search metric to use: euclidean / descriptor
  std::string metric;
};

}; // namespace wildSLAM
