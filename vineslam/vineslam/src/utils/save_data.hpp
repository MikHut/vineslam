#pragma once

#include <iostream>
#include <ctime>
#include <fstream>
#include <pf.hpp>

const std::string folder = "/home/andre-criis/Source/vineslam_data/raw_data";

namespace vineslam
{

void saveParticleClusters(const std::map<int, Gaussian<pose, pose>>& gauss_map,
                          const std::vector<Particle>&               particles,
                          const int&                                 n_clusters,
                          const int&                                 it)
{
  std::ofstream file;
  file.open(folder + "/clusters/" + "clusters_" + std::to_string(it) + ".txt");
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

void saveRobotPath(const std::vector<pose>& gt, const std::vector<pose>& rp)
{
  std::time_t t = std::time(0);
  std::ofstream file;
  file.open(folder + "/paths/" + "path_" + std::to_string(t) + ".txt");

  for (const auto& m_pose : gt) {
    file << m_pose.x << " " << m_pose.y << " " << m_pose.z << " ";
  }
  file << "\n";

  for (const auto& m_pose : rp) {
    file << m_pose.x << " " << m_pose.y << " " << m_pose.z << " ";
  }
  file << "\n";

  file.close();
}

} // namespace vineslam
