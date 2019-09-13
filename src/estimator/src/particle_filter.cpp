#include "../include/particle_filter.hpp"

ParticleFilter::ParticleFilter() {}

void ParticleFilter::init()
{
  int    id     = 0;
  double x      = 0.0;
  double y      = 0.0;
  double theta  = -MAX_DTHETA / 2 * RAD;
  double weight = 1.0;

  while (x <= MAX_DXY) {
    while (y <= MAX_DXY) {
      while (theta <= MAX_DTHETA / 2) {
	      particles.push_back(Particle<double>(id, Point<double>(x, y), theta, weight));
	      theta += RAD;
	      id++;
      }
      y += 0.01;
    }
    x += 0.01;
  }
}
