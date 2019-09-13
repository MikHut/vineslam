#pragma once

#include <iostream>
#include "utils.hpp"

class ParticleFilter
{
public:
  ParticleFilter();
  void init();

  std::vector<Particle<double>> particles;

private:
};
