#pragma once

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

  // Access function - returns the ith element of both landmark and features arrays
  std::pair<Landmark, Feature> operator()(int i)
  {
    if (i >= landmarks.size() || i >= features.size()) {
      std::cout << "ERROR: Access to cell members out of bounds." << std::endl;
      return std::pair<Landmark, Feature>();
    }
    return std::pair<Landmark, Feature>(landmarks[i], features[i]);
  }

  // Inserts a landmark in the landmarks array
  void insert(const Landmark& m_landmark) { landmarks.push_back(m_landmark); }

  // Inserts a feature in the features array
  void insert(const Feature& m_feature) { features.push_back(m_feature); }

private:
  // List of landmarks and features at each cell
  std::vector<Landmark> landmarks;
  std::vector<Feature>  features;
};

class OccupancyMap
{
public:
  // Class constructor
  // - initializes the grid map given the input parameters
  OccupancyMap(const std::string& config_path);

  // 2D grid map access operators
  Cell operator()(int i, int j)
  {
    if (i * width + j >= m_gmap.size()) {
      std::cout << "ERROR: Access to grid map member out of bounds." << std::endl;
      return Cell();
    } else
      return m_gmap[i * width + j];
  }

  // Insert a Cell method
  bool insert(const Cell& m_cell, const int& i, const int& j);

  // Method to get all the adjacent cells to a given cell
  bool getAdjacent(const Cell&          m_cell,
                   const int&           i,
                   const int&           j,
                   std::array<Cell, 8>& adjacent);

private:
  // Private grid map to store all the cells
  std::vector<Cell> m_gmap;

  // Grid map dimensions
  int width;
  int height;
};
