#pragma once

#include <iostream>
#include <ctime>
#include <fstream>
#include <pf.hpp>

const std::string folder = "/home/andre-criis/Source/vineslam_data/"
                           "/raw_data";

namespace vineslam
{

static void
saveParticleClusters(const std::map<int, Gaussian<pose, pose>>& gauss_map,
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

static void saveRobotPathTUM(const std::vector<int>&                  timestamps,
                             const std::vector<std::array<float, 7>>& gps_path,
                             const std::vector<std::array<float, 7>>& odom_path,
                             const std::vector<std::array<float, 7>>& robot_path)
{
  std::time_t   t = std::time(0);
  std::ofstream file;

  file.open(folder + "/paths/" + "path_vineslam_" + std::to_string(t) + ".txt");
  for (size_t i = 0; i < timestamps.size(); i++) {
    file << timestamps[i] << " " << robot_path[i][0] << " " << robot_path[i][1]
         << " " << robot_path[i][2] << " " << robot_path[i][3] << " "
         << robot_path[i][4] << " " << robot_path[i][5] << " " << robot_path[i][6]
         << "\n";
  }
  file.close();

  file.open(folder + "/paths/" + "path_gps_" + std::to_string(t) + ".txt");
  for (size_t i = 0; i < timestamps.size(); i++) {
    file << timestamps[i] << " " << gps_path[i][0] << " " << gps_path[i][1] << " "
         << gps_path[i][2] << " " << gps_path[i][3] << " " << gps_path[i][4] << " "
         << gps_path[i][5] << " " << gps_path[i][6] << "\n";
  }
  file.close();

  file.open(folder + "/paths/" + "path_odom_" + std::to_string(t) + ".txt");
  for (size_t i = 0; i < timestamps.size(); i++) {
    file << timestamps[i] << " " << odom_path[i][0] << " " << odom_path[i][1] << " "
         << odom_path[i][2] << " " << odom_path[i][3] << " " << odom_path[i][4]
         << " " << odom_path[i][5] << " " << odom_path[i][6] << "\n";
  }
  file.close();
}

static void saveRobotPathKitti(const std::vector<TF>& gps_path,
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
  file.close();

  file.open(folder + "/paths/" + "path_gps_" + std::to_string(t) + ".txt");
  for (const auto& m_tf : gps_path) {
    file << m_tf.R[0] << " " << m_tf.R[1] << " " << m_tf.R[2] << " " << m_tf.t[0]
         << " " << m_tf.R[3] << " " << m_tf.R[4] << " " << m_tf.R[5] << " "
         << m_tf.t[1] << " " << m_tf.R[6] << " " << m_tf.R[7] << " " << m_tf.R[8]
         << " " << m_tf.t[2] << "\n";
  }
  file.close();

  file.open(folder + "/paths/" + "path_odom" + std::to_string(t) + ".txt");
  for (const auto& m_tf : odom_path) {
    file << m_tf.R[0] << " " << m_tf.R[1] << " " << m_tf.R[2] << " " << m_tf.t[0]
         << " " << m_tf.R[3] << " " << m_tf.R[4] << " " << m_tf.R[5] << " "
         << m_tf.t[1] << " " << m_tf.R[6] << " " << m_tf.R[7] << " " << m_tf.R[8]
         << " " << m_tf.t[2] << "\n";
  }
  file.close();
}

} // namespace vineslam
