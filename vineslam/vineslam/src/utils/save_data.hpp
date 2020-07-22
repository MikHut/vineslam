#pragma once

#include <iostream>
#include <fstream>

const std::string folder = "/home/andre-criis/Source/vineslam_data/";

namespace vineslam
{

void saveParticleClusters(const std::map<int, Gaussian<pose, pose>>& gauss_map,
                          const std::vector<Particle>&               particles,
                          const int&                                 n_clusters,
                          const int&                                 it)
{
  std::ofstream file;
  file.open(folder + "clusters_" + std::to_string(it) + ".txt");
  for (const auto& gauss : gauss_map) {
    std::vector<float> x, y, z;
    file << gauss.second.mean.x << " " << gauss.second.mean.y << " "
         << gauss.second.mean.z << " ";
    for (const auto& particle : particles) {
      if (particle.which_cluster == gauss.first) {
        file << particle.p.x << " " << particle.p.y << " " << particle.p.z << " ";
      }
    }
    file << "\n";
  }

  file.close();
}

} // namespace vineslam
