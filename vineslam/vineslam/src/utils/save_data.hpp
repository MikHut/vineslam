#pragma once

#include <iostream>
#include <ctime>
#include <fstream>
#include <pf.hpp>

const std::string folder = "/home/andre-criis/Source/vineslam_data/"
                           "08-2020-paper-clusters/raw_data";

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

void saveRobotPath(const std::vector<TF>& gps_path,
                   const std::vector<TF>& odom_path,
                   const std::vector<TF>& robot_path)
{
  std::time_t   t = std::time(0);
  std::ofstream file;

  file.open(folder + "/paths/" + "path_vineslam_" + std::to_string(t) + ".txt");
  for (const auto& m_tf : robot_path) {
    file << m_tf.R[0] << " " << m_tf.R[1] << " " << m_tf.R[2] << " " << m_tf.t[0]
         << " " << m_tf.R[3] << " " << m_tf.R[4] << " " << m_tf.R[5] << " "
         << m_tf.t[1] << " " << m_tf.R[6] << " " << m_tf.R[7] << " " << m_tf.R[8]
         << " " << m_tf.t[2] << "\n";
  }
  file << "\n";
  file.close();

  file.open(folder + "/paths/" + "path_gps_" + std::to_string(t) + ".txt");
  for (const auto& m_tf : gps_path) {
    file << m_tf.R[0] << " " << m_tf.R[1] << " " << m_tf.R[2] << " " << m_tf.t[0]
         << " " << m_tf.R[3] << " " << m_tf.R[4] << " " << m_tf.R[5] << " "
         << m_tf.t[1] << " " << m_tf.R[6] << " " << m_tf.R[7] << " " << m_tf.R[8]
         << " " << m_tf.t[2] << "\n";
  }
  file << "\n";
  file.close();

  file.open(folder + "/paths/" + "path_odom_" + std::to_string(t) + ".txt");
  for (const auto& m_tf : odom_path) {
    file << m_tf.R[0] << " " << m_tf.R[1] << " " << m_tf.R[2] << " " << m_tf.t[0]
         << " " << m_tf.R[3] << " " << m_tf.R[4] << " " << m_tf.R[5] << " "
         << m_tf.t[1] << " " << m_tf.R[6] << " " << m_tf.R[7] << " " << m_tf.R[8]
         << " " << m_tf.t[2] << "\n";
  }
  file << "\n";
  file.close();
}

} // namespace vineslam
