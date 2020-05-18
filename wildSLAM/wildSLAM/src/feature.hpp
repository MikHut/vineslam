#pragma once

#include <iostream>
#include "math/point3D.hpp"

class Feature
{
public:
  Feature() = default;
  // Class constructor
  // - initializes its image position and feature type
  Feature(const int& u, const int& v, const std::string& type)
  {
    (*this).u    = u;
    (*this).v    = v;
    (*this).r    = 255;
    (*this).g    = 255;
    (*this).b    = 255;
    (*this).type = type;
  }

  // Class constructor
  // - initializes its image position, color, and feature type
  Feature(const int&         u,
          const int&         v,
          const uint8_t&     r,
          const uint8_t&     g,
          const uint8_t&     b,
          const std::string& type)
  {
    (*this).u    = u;
    (*this).v    = v;
    (*this).r    = r;
    (*this).g    = g;
    (*this).b    = b;
    (*this).type = type;
  }

  // Class constructor
  // - initializes its image/world position and feature type
  Feature(const int& u, const int& v, const point3D& pos, const std::string& type)
  {
    (*this).u    = u;
    (*this).v    = v;
    (*this).r    = 255;
    (*this).g    = 255;
    (*this).b    = 255;
    (*this).pos  = pos;
    (*this).type = type;
  }

  // Class constructor
  // - initializes its image/world position, color, and feature type
  Feature(const int&         u,
          const int&         v,
          const uint8_t&     r,
          const uint8_t&     g,
          const uint8_t&     b,
          const point3D&     pos,
          const std::string& type)
  {
    (*this).u    = u;
    (*this).v    = v;
    (*this).r    = r;
    (*this).g    = g;
    (*this).b    = b;
    (*this).pos  = pos;
    (*this).type = type;
  }

  // Print semantic landmark information
  void print() const
  {
    std::cout << "Feature " << std::endl;
    std::cout << " type:              " << type << std::endl;
    std::cout << " image position:   [" << u << "," << v << "]" << std::endl;
    std::cout << " world position:    " << pos << std::endl;
  }

  // Image pixel position
  int u;
  int v;
  // RGB info
  uint8_t r;
  uint8_t g;
  uint8_t b;
  // World 3D position
  point3D pos;
  // Feature type
  std::string type;

private:
};
