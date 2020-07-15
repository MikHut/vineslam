#pragma once

#include <iostream>
#include <fstream>

const std::string folder = "/home/andre-criis/Source/vineslam_data/";

namespace vineslam
{

void saveParticleClusters(const std::vector<Particle>& particles,
                          const int&                   n_clusters,
                          const int&                   it)
{
  std::ofstream file;
  file.open(folder + "clusters_" + std::to_string(it) + ".txt");
  for (int i = 0; i < n_clusters; i++) {
    std::vector<float> x, y, z;
    file << i << " ";
    for (const auto& particle : particles) {
      if (particle.which_cluster == i) {
        file << particle.p.x << " " << particle.p.y << " " << particle.p.z << " ";
      }
    }
    file << "\n";
  }

  file.close();
}

} // namespace vineslam
