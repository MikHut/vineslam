#pragma once

#include <iostream>
#include "math/point3D.hpp"

class Feature
{
public:
  Feature() {}
  // Class constructor
  // - initializes its image position and feature type
  Feature(const int& u, const int& v, const std::string& type)
  {
    (*this).u    = u;
    (*this).v    = v;
    (*this).type = type;
  }

  // Class constructor
  // - initializes its image/world position and feature type
  Feature(const int& u, const int& v, const point3D& pos, const std::string& type)
  {
    (*this).u    = u;
    (*this).v    = v;
    (*this).pos  = pos;
    (*this).type = type;
  }

  // Print semantic landmark information
  void print()
  {
    std::cout << "Feature " << std::endl;
    std::cout << " type:              " << type << std::endl;
    std::cout << " image position:   [" << u << "," << v << "]" << std::endl;
    std::cout << " world position:    " << pos << std::endl;
  }

  // Image pixel position
  int u;
  int v;
  // World 3D position
  point3D pos;
  // Feature type
  std::string type;

private:
};
