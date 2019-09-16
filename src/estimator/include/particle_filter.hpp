#pragma once

#include "landmark_processor.hpp"
#include "utils.hpp"
#include <iostream>

class ParticleFilter
{
public:
  ParticleFilter(const Parameters& params);
  void init();
  void process(const std::vector<Point<double>>& poses);

  LandmarkProcessor*		landm_obj;
  std::vector<Particle<double>> particles;

private:
  void updateWeights();

  Parameters params;
};
