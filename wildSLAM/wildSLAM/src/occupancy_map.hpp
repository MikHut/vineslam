#pragma once

#include "math/point3D.hpp"
#include <landmark.hpp>
#include <feature.hpp>

#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>

class Cell
{
public:
  // Default constructor
  Cell() {}

  // Class constructor
  // - initializes the cell containers
  Cell(const std::vector<Landmark>& landmarks, const std::vector<Feature>& features)
  {
    (*this).landmarks = landmarks;
    (*this).features  = features;
  }

  // Access function - returns the ith/jth element of both landmark and features
  // arrays. If i or j negative, returns a default constructor
  std::pair<Landmark, Feature> operator()(int i, int j)
  {
    if (i >= landmarks.size() || j >= features.size()) {
      std::cout << "ERROR: Access to cell members out of bounds." << std::endl;
      return std::pair<Landmark, Feature>();
    } else {
      Landmark m_landmark;
      Feature  m_feature;
      if (i < 0 && j < 0) {
        m_landmark = Landmark();
        m_feature  = Feature();
      } else if (i >= 0 && j < 0) {
        m_landmark = landmarks[i];
        m_feature  = Feature();
      } else if (i < 0 && j >= 0) {
        m_landmark = Landmark();
        m_feature  = features[j];
      } else {
        m_landmark = landmarks[i];
        m_feature  = features[j];
      }
      return std::pair<Landmark, Feature>(m_landmark, m_feature);
    }
  }

  // Inserts a landmark in the landmarks array
  void insert(const Landmark& m_landmark) { landmarks.push_back(m_landmark); }

  // Inserts a feature in the features array
  void insert(const Feature& m_feature) { features.push_back(m_feature); }

  // List of landmarks and features at each cell
  std::vector<Landmark> landmarks;
  std::vector<Feature>  features;

private:
};

class OccupancyMap
{
public:
  // Class constructor
  // - initializes the grid map given the input parameters
  OccupancyMap(const std::string& config_path);

  // 2D grid map direct access to cell coordinates
  Cell& operator()(int i, int j)
  {
    // Compute indexes having into account that grid map considers negative values
    int m_i = i - (origin.x / resolution);
    int m_j = j - (origin.y / resolution);
    // Verify validity of indexing
    if (m_i + (m_j * (width / resolution)) >= m_gmap.size()) {
      std::cout << "ERROR: Access to grid map member (" << i << "," << j
                << ") out of bounds." << std::endl;
    }
    return m_gmap[m_i + m_j * (width / resolution)];
  }

  // 2D grid map access given a Feature/Landmark location
  Cell& operator()(float i, float j)
  {
    int m_i = std::round(i / resolution);
    int m_j = std::round(j / resolution);

    return (*this)(m_i, m_j);
  }

  // Define iterator to provide access to the cells array
  typedef std::vector<Cell>::iterator iterator;
  // Return members to provide access tothe cells array
  iterator begin() { return m_gmap.begin(); }
  iterator end() { return m_gmap.end(); }

  // Insert a Landmark using the direct grid coordinates
  bool insert(const Landmark& m_landmark, const int& i, const int& j);
  // Insert a Landmark given a Feature/Landmark location
  bool insert(const Landmark& m_landmark, const float& i, const float& j);

  // Insert a Feature using the direct grid coordinates
  bool insert(const Feature& m_feature, const int& i, const int& j);
  // Insert a Feature given a Feature/Landmark location
  bool insert(const Feature& m_feature, const float& i, const float& j);

  // Since Landmark map is built with a KF, Landmarks position change in each
  // iteration. This routine updates the position of a given Landmark
  bool update(const Landmark& old_landmark,
              const Landmark& new_landmark,
              const float&    i,
              const float&    j);

  // Method to get all the adjacent cells to a given cell
  bool getAdjacent(const Cell&          m_cell,
                   const int&           i,
                   const int&           j,
                   std::array<Cell, 8>& adjacent);

  // Method to get all the adjacent cells to a given cell given a Feature/Landmark
  // location
  bool getAdjacent(const Cell&          m_cell,
                   const float&         i,
                   const float&         j,
                   std::array<Cell, 8>& adjacent);

private:
  // Private grid map to store all the cells
  std::vector<Cell> m_gmap;

  // Grid map dimensions
  // NOTE: corners are in reference to the given origin
  point3D origin;
  float   width;
  float   height;
  float   resolution;
};
