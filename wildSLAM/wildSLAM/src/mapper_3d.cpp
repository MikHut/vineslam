#include "mapper_3d.hpp"

Mapper3D::Mapper3D(const Parameters& params) : params(params) {}

void Mapper3D::init()
{
}

void Mapper3D::process(const float* depths)
{
  for(int i = 0; i < params.width; i++) {
    for( int j = 0; j < params.height; j++) {
      int idx = i + params.width * j;
      std::cout << depths[idx] << ",";
    }
  }
  std::cout << std::endl;
}
