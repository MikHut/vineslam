#include "pf.hpp"

#include <cmath>
#include <utils/save_data.hpp>

namespace vineslam
{

PF::PF(const Parameters& params, const pose& initial_pose)
{
  // - General parameters
  use_landmarks    = params.use_landmarks;
  use_corners      = params.use_corners;
  use_planars      = params.use_planars;
  use_planes       = params.use_planes;
  use_ground_plane = params.use_ground_plane;
  use_icp          = params.use_icp;
  use_gps          = params.use_gps;
  n_particles      = static_cast<float>(params.number_particles);
  // - Motion model parameters
  alpha1 = params.alpha1;
  alpha2 = params.alpha2;
  alpha3 = params.alpha3;
  alpha4 = params.alpha4;
  // - Innovation parameters
  sigma_xy    = params.sigma_xy;
  sigma_z     = params.sigma_z;
  sigma_roll  = params.sigma_roll * DEGREE_TO_RAD;
  sigma_pitch = params.sigma_pitch * DEGREE_TO_RAD;
  sigma_yaw   = params.sigma_yaw * DEGREE_TO_RAD;
  // - Update standard deviations of each layer
  sigma_landmark_matching = params.sigma_landmark_matching;
  sigma_feature_matching  = params.sigma_feature_matching;
  sigma_corner_matching   = params.sigma_corner_matching;
  sigma_planar_matching   = params.sigma_planar_matching;
  sigma_planes            = params.sigma_planes;
  sigma_gps               = params.sigma_gps;
  // - Set clustering parameters
  k_clusters   = params.number_clusters;
  k_iterations = 20;

  // Initialize and set ICP parameters
  icp = new ICP<ImageFeature>(params);

  // Initialize normal distributions
  particles.resize(n_particles);

  // Initialize all particles
  for (size_t i = 0; i < particles.size(); i++) {
    // Calculate the initial pose for each particle considering
    // - the input initial pose
    // - a sample distribution to spread the particles
    pose m_pose = initial_pose + pose(sampleGaussian(sigma_xy),
                                      sampleGaussian(sigma_xy),
                                      sampleGaussian(sigma_z * 2),
                                      sampleGaussian(sigma_roll * 2),
                                      sampleGaussian(sigma_pitch * 2),
                                      sampleGaussian(sigma_yaw));
    // Compute initial weight of each particle
    float weight = 1.;
    // Insert the particle into the particles array
    particles[i] = Particle(i, m_pose, weight);
  }

  // Iteration number
  n_it = 0;
}

void PF::motionModel(const pose& odom_inc, const pose& p_odom)
{
  // Input parameters
  float m_xx_noise = 0.2;
  float m_yy_noise = 0.2;
  float m_zz_noise = 0.1;
  float m_RR_noise = 0.5;
  float m_PP_noise = 0.5;
  float m_YY_noise = 0.8;

  float d_trans = odom_inc.norm3D();

  // Innovate particles
  for (auto& particle : particles) {
    // Build pose noise transformation matrix
    pose pose_noise(
        m_xx_noise, m_yy_noise, m_zz_noise, m_RR_noise, m_PP_noise, m_YY_noise);

    std::array<float, 6> gaussian_noise{};
    for (float& i : gaussian_noise) i = d_trans * sampleGaussian(1.0);

    pose_noise.x *= gaussian_noise[0];     // xx
    pose_noise.y *= gaussian_noise[1];     // yy
    pose_noise.z *= gaussian_noise[2];     // zz
    pose_noise.roll *= gaussian_noise[3];  // RR
    pose_noise.pitch *= gaussian_noise[4]; // PP
    pose_noise.yaw *= gaussian_noise[5];   // YY

    std::array<float, 9> R_noise{};
    pose_noise.toRotMatrix(R_noise);
    TF odom_noise_tf(R_noise,
                     std::array<float, 3>{pose_noise.x, pose_noise.y, pose_noise.z});

    // Built odom increment transformation matrix
    std::array<float, 9> R_inc{};
    odom_inc.toRotMatrix(R_inc);
    TF odom_inc_tf(R_inc, std::array<float, 3>{odom_inc.x, odom_inc.y, odom_inc.z});

    // Final pose increment
    TF innovation_tf = odom_inc_tf * odom_noise_tf;

    // Save info for the next iteration
    particle.pp  = particle.p;
    particle.ptf = particle.tf;

    // Apply transformation
    particle.tf = particle.ptf * innovation_tf;
    particle.p  = pose(particle.tf.R, particle.tf.t);
  }
}

void PF::update(const std::vector<SemanticFeature>& landmarks,
                const std::vector<Corner>&          corners,
                const std::vector<Planar>&          planars,
                const std::vector<Plane>&           planes,
                const Plane&                        ground_plane,
                const std::vector<ImageFeature>&    surf_features,
                const pose&                         gps_pose,
                OccupancyMap*                       grid_map)
{
  std::vector<float> semantic_weights(n_particles, 0.);
  std::vector<float> corner_weights(n_particles, 0.);
  std::vector<float> planar_weights(n_particles, 0.);
  std::vector<float> planes_weights(n_particles, 0.);
  std::vector<float> ground_weights(n_particles, 0.);
  std::vector<float> surf_weights(n_particles, 0.);
  std::vector<float> gps_weights(n_particles, 0.);

  logs = "\n";

  auto before = std::chrono::high_resolution_clock::now();
  if (use_landmarks)
    highLevel(landmarks, grid_map, semantic_weights);
  auto after = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float, std::milli> duration = after - before;
  logs += "Time elapsed on PF - high-level features (msecs): " +
          std::to_string(duration.count()) + "\n";
  before = std::chrono::high_resolution_clock::now();
  if (use_corners)
    mediumLevelCorners(corners, grid_map, corner_weights);
  after    = std::chrono::high_resolution_clock::now();
  duration = after - before;
  logs += "Time elapsed on PF - corner features (msecs): " +
          std::to_string(duration.count()) + " (" + std::to_string(corners.size()) +
          ")\n";
  before = std::chrono::high_resolution_clock::now();
  if (use_planars)
    mediumLevelPlanars(planars, grid_map, planar_weights);
  after    = std::chrono::high_resolution_clock::now();
  duration = after - before;
  logs += "Time elapsed on PF - planar features (msecs): " +
          std::to_string(duration.count()) + +" (" + std::to_string(planars.size()) +
          ")\n";
  before = std::chrono::high_resolution_clock::now();
  if (use_icp)
    lowLevel(surf_features, grid_map, surf_weights);
  after    = std::chrono::high_resolution_clock::now();
  duration = after - before;
  logs +=
      "Time elapsed on PF - icp (msecs): " + std::to_string(duration.count()) + "\n";
  before = std::chrono::high_resolution_clock::now();
  if (use_gps)
    gps(gps_pose, gps_weights);
  after    = std::chrono::high_resolution_clock::now();
  duration = after - before;
  logs +=
      "Time elapsed on PF - gps (msecs): " + std::to_string(duration.count()) + "\n";

  // Multi-modal weights normalization
  float semantic_max =
      *std::max_element(semantic_weights.begin(), semantic_weights.end());
  float corners_max =
      *std::max_element(corner_weights.begin(), corner_weights.end());
  float planars_max =
      *std::max_element(planar_weights.begin(), planar_weights.end());
  float planes_max = *std::max_element(planes_weights.begin(), planes_weights.end());
  float ground_max = *std::max_element(ground_weights.begin(), ground_weights.end());
  float surf_max   = *std::max_element(surf_weights.begin(), surf_weights.end());
  float gps_max    = *std::max_element(gps_weights.begin(), gps_weights.end());
  for (auto& particle : particles) {
    float m_lw =
        (semantic_max > 0.) ? semantic_weights[particle.id] : static_cast<float>(1.);
    float m_cw =
        (corners_max > 0.) ? corner_weights[particle.id] : static_cast<float>(1.);
    float m_rw =
        (planars_max > 0.) ? planar_weights[particle.id] : static_cast<float>(1.);
    float m_pw =
        (planes_max > 0.) ? planes_weights[particle.id] : static_cast<float>(1.);
    float m_gw =
        (ground_max > 0.) ? ground_weights[particle.id] : static_cast<float>(1.);
    float m_sw =
        (surf_max > 0.) ? surf_weights[particle.id] : static_cast<float>(1.);
    float m_gpsw =
        (gps_max > 0.) ? gps_weights[particle.id] : static_cast<float>(1.);

    particle.w = m_lw * m_cw * m_rw * m_pw * m_gw * m_sw * m_gpsw;
    w_sum += particle.w;
  }

  n_it++;
}

void PF::gps(const pose& gps_pose, std::vector<float>& ws)
{
  float normalizer_gps = static_cast<float>(1.) / (sigma_gps * std::sqrt(M_2PI));

  for (const auto& particle : particles) {
    // - GPS [x, y] weight
    float w_gps;
    float dist = particle.p.distance(gps_pose);
    w_gps = (normalizer_gps * static_cast<float>(std::exp(-1. / sigma_gps * dist)));

    ws[particle.id] = w_gps;
  }
}

void PF::highLevel(const std::vector<SemanticFeature>& landmarks,
                   OccupancyMap*                       grid_map,
                   std::vector<float>&                 ws)
{
  float normalizer_landmark =
      static_cast<float>(1.) / (sigma_landmark_matching * std::sqrt(M_2PI));

  // Loop over all particles
  for (const auto& particle : particles) {
    // Convert particle orientation to rotation matrix
    pose m_pose  = particle.p;
    m_pose.roll  = 0.;
    m_pose.pitch = 0.;
    m_pose.z     = 0.;
    std::array<float, 9> Rot{};
    m_pose.toRotMatrix(Rot);

    // ------------------------------------------------------
    // --- 2D semantic feature map fitting
    // ------------------------------------------------------
    std::vector<float> dlandmarkvec;
    for (const auto& landmark : landmarks) {
      // Convert landmark to the maps's referential frame
      point X;
      X.x = landmark.pos.x * Rot[0] + landmark.pos.y * Rot[1] +
            landmark.pos.z * Rot[2] + m_pose.x;
      X.y = landmark.pos.x * Rot[3] + landmark.pos.y * Rot[4] +
            landmark.pos.z * Rot[5] + m_pose.y;
      X.z = 0.;

      // Search for a correspondence in the current cell first
      float best_correspondence = std::numeric_limits<float>::max();
      bool  found               = false;
      for (const auto& m_landmark : (*grid_map)(X.x, X.y, 0).landmarks) {
        float dist_min = X.distanceXY(m_landmark.second.pos);

        if (dist_min < best_correspondence) {
          best_correspondence = dist_min;
          found               = true;
        }
      }

      // Only search in the adjacent cells if we do not find in the source cell
      if (!found) {
        std::vector<Cell> adjacents;
        grid_map->getAdjacent(X.x, X.y, 0, 2, adjacents);
        for (const auto& m_cell : adjacents) {
          for (const auto& m_landmark : m_cell.landmarks) {
            float dist_min = X.distanceXY(m_landmark.second.pos);
            if (dist_min < best_correspondence) {
              best_correspondence = dist_min;
              found               = true;
            }
          }
        }
      }

      // Save distance if a correspondence was found
      if (!found)
        continue;
      else
        dlandmarkvec.push_back(best_correspondence);
    }

    // - Semantic landmark matching [x, y, yaw] weight
    float w_landmarks = 1.;
    if (dlandmarkvec.size() <= 1) {
      w_landmarks = 0.;
    } else {
      for (const auto& dist : dlandmarkvec)
        w_landmarks +=
            (normalizer_landmark *
             static_cast<float>(std::exp(-1. / sigma_landmark_matching * dist)));
    }

    ws[particle.id] = w_landmarks;
  }
}

void PF::mediumLevelCorners(const std::vector<Corner>& corners,
                            OccupancyMap*              grid_map,
                            std::vector<float>&        ws)
{
  float normalizer_corner =
      static_cast<float>(1.) / (sigma_corner_matching * std::sqrt(M_2PI));

  // Loop over all particles
  for (const auto& particle : particles) {
    // ------------------------------------------------------
    // --- 3D corner map fitting
    // ------------------------------------------------------
    float              w_corners = 0;
    std::vector<float> dcornervec;
    for (const auto& corner : corners) {
      // Convert feature to the map's referential frame
      point X = corner.pos * particle.tf;

      std::vector<Corner> m_corners = (*grid_map)(X.x, X.y, X.z).corner_features;

      // Search for a correspondence in the current cell first
      float best_correspondence = 0.5;
      bool  found               = false;
      for (const auto& m_corner : m_corners) {
        float dist_sq = ((X.x - m_corner.pos.x) * (X.x - m_corner.pos.x) +
                         (X.y - m_corner.pos.y) * (X.y - m_corner.pos.y) +
                         (X.z - m_corner.pos.z) * (X.z - m_corner.pos.z));

        if (dist_sq < best_correspondence) {
          best_correspondence = dist_sq;
          found               = true;
        }
      }

      // Save distance if a correspondence was found
      if (found)
        w_corners += (normalizer_corner *
                      static_cast<float>(std::exp(-1. / sigma_corner_matching *
                                                  best_correspondence)));
    }

    ws[particle.id] = w_corners;
  }
}

void PF::mediumLevelPlanars(const std::vector<Planar>& planars,
                            OccupancyMap*              grid_map,
                            std::vector<float>&        ws)
{
  float normalizer_planar =
      static_cast<float>(1.) / (sigma_planar_matching * std::sqrt(M_2PI));

  // Loop over all particles
  for (const auto& particle : particles) {
    // ------------------------------------------------------
    // --- 3D planar map fitting
    // ------------------------------------------------------
    float w_planars = 0.;
    for (const auto& planar : planars) {
      // Convert feature to the map's referential frame
      point X = planar.pos * particle.tf;

      std::vector<Planar> m_planars = (*grid_map)(X.x, X.y, X.z).planar_features;

      // Search for a correspondence in the current cell first
      float best_correspondence = 0.5;
      bool  found               = false;
      for (const auto& m_planar : m_planars) {
        float dist_sq = ((X.x - m_planar.pos.x) * (X.x - m_planar.pos.x) +
                         (X.y - m_planar.pos.y) * (X.y - m_planar.pos.y) +
                         (X.z - m_planar.pos.z) * (X.z - m_planar.pos.z));

        if (dist_sq < best_correspondence) {
          best_correspondence = dist_sq;
          found               = true;
        }
      }

      // Save distance if a correspondence was found
      if (found)
        w_planars += (normalizer_planar *
                      static_cast<float>(std::exp((-1. / sigma_planar_matching) *
                                                  best_correspondence)));
    }

    ws[particle.id] = w_planars;
  }
}

void PF::lowLevel(const std::vector<ImageFeature>& surf_features,
                  OccupancyMap*                    grid_map,
                  std::vector<float>&              ws)
{
  // ------------------------------------------------------------------------------
  // ---------------- Cluster particles
  // ------------------------------------------------------------------------------
  std::map<int, Gaussian<pose, pose>> gauss_map;
  cluster(gauss_map);

  // ------------------------------------------------------------------------------
  // ---------------- Scan match
  // ------------------------------------------------------------------------------
  scanMatch(surf_features, grid_map, gauss_map, ws);
}

void PF::cluster(std::map<int, Gaussian<pose, pose>>& gauss_map)
{
  // -------------------------------------------------------------------------------
  // ------ (1) Initialize the clusters - kmean++
  // ------ (1) from: https://www.geeksforgeeks.org/ml-k-means-algorithm/
  // -------------------------------------------------------------------------------
  // -- Select first centroid randomly
  std::vector<pose> centroids;
  std::vector<pose> m_centroids;
  int               n = particles.size();
  srand(time(nullptr)); // need to set the random seed
  centroids.push_back(particles[rand() % n].p);
  // -- Compute remaining k - 1 centroids
  for (int i = 1; i < k_clusters; i++) {
    std::vector<float> min_dists;
    for (const auto& particle : particles) {
      // Find minimum distance to already computed centroid
      float min_dist = std::numeric_limits<float>::max();
      for (auto& centroid : centroids) {
        float m_dist = particle.p.distance(centroid);
        if (m_dist < min_dist) {
          min_dist = m_dist;
        }
      }
      min_dists.push_back(min_dist);
    }
    // Select data point with maximum distance as our next centroid
    auto it  = std::max_element(min_dists.begin(), min_dists.end());
    int  idx = std::distance(min_dists.begin(), it);
    centroids.push_back(particles[idx].p);
  }
  // -- Initialize equal-sized cluster
  std::vector<std::map<int, float>> heap(k_clusters);
  for (int i = 0; i < k_clusters; i++) {
    for (const auto& particle : particles) {
      float dist           = centroids[i].distance(particle.p);
      heap[i][particle.id] = dist;
    }
  }
  std::vector<int> num_per_cluster(k_clusters, 0);
  int              max_num = static_cast<int>(static_cast<float>(n_particles) /
                                 static_cast<float>(k_clusters));
  for (auto& particle : particles) {
    float min_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < k_clusters; i++) {
      float dist = heap[i][particle.id];
      if (dist < min_dist && num_per_cluster[i] < max_num) {
        particle.which_cluster = i;
        min_dist               = dist;
      }
    }
    num_per_cluster[particle.which_cluster]++;
  }

  int swaps = 0;
  for (int i = 0; i < k_iterations; i++) {

    // -----------------------------------------------------------------------------
    // ------ (2) Re-compute the centroids
    // -----------------------------------------------------------------------------
    num_per_cluster = std::vector<int>(k_clusters, 0);
    for (const auto& particle : particles) {
      centroids[particle.which_cluster].x += particle.p.x;
      centroids[particle.which_cluster].y += particle.p.y;
      centroids[particle.which_cluster].z += particle.p.z;

      num_per_cluster[particle.which_cluster]++;
    }

    for (int j = 0; j < k_clusters; j++) {
      if (num_per_cluster[j] == 0) {
        centroids[j] = pose(0., 0., 0., 0., 0., 0.);
        continue;
      } else {
        centroids[j].x /= num_per_cluster[j];
        centroids[j].y /= num_per_cluster[j];
        centroids[j].z /= num_per_cluster[j];
      }
    }

    // -----------------------------------------------------------------------------
    // ------ (3) Assign the particles to clusters
    // ------ (3) Equal-size K-means from https://stackoverflow.com/a/8810231
    // -----------------------------------------------------------------------------
    std::map<int, int> swap;
    for (auto& particle : particles) {
      float min_dist      = std::numeric_limits<float>::max();
      int   which_cluster = -1;
      for (int cluster = 0; cluster < k_clusters; cluster++) {
        float m_dist = particle.p.distance(centroids[cluster]);
        if (m_dist < min_dist) {
          min_dist      = m_dist;
          which_cluster = cluster;
        }
      }

      if (which_cluster != particle.which_cluster) {
        auto it = swap.find(particle.which_cluster);
        if (num_per_cluster[particle.which_cluster] >
            static_cast<float>(n_particles) / static_cast<float>(k_clusters)) {
          num_per_cluster[which_cluster]++;
          num_per_cluster[particle.which_cluster]--;
          particle.which_cluster = which_cluster;
          swaps++;
        } else if (it != swap.end() &&
                   particles[it->second].which_cluster == which_cluster &&
                   it->second != particle.id) {
          particles[it->second].which_cluster = particle.which_cluster;
          particle.which_cluster              = which_cluster;
          swap.erase(particle.which_cluster);
          swaps++;
        } else {
          swap[which_cluster] = particle.id;
        }
      }
    }

    if (swaps == 0) {
      break;
    } else {
      swaps = 0;
    }
  }

  // -----------------------------------------------------------------------------
  // ------ (4) Compute weighted Gaussian approximation of each cluster
  // -----------------------------------------------------------------------------
  // ---- Weighted mean
  // - For the orientations - Mean of circular quantities
  // (https://en.wikipedia.org/wiki/Mean_of_circular_quantities)
  std::vector<pose>  means(k_clusters, pose(0., 0., 0., 0., 0., 0.));
  std::vector<pose>  means_sin(k_clusters, pose(0., 0., 0., 0., 0., 0.));
  std::vector<pose>  means_cos(k_clusters, pose(0., 0., 0., 0., 0., 0.));
  std::vector<float> sums(k_clusters, 0.);
  for (const auto& particle : particles) {
    means[particle.which_cluster].x += (particle.w * particle.p.x);
    means[particle.which_cluster].y += (particle.w * particle.p.y);
    means[particle.which_cluster].z += (particle.w * particle.p.z);
    means_sin[particle.which_cluster].roll +=
        (particle.w * std::sin(particle.p.roll));
    means_cos[particle.which_cluster].roll +=
        (particle.w * std::cos(particle.p.roll));
    means_sin[particle.which_cluster].pitch +=
        (particle.w * std::sin(particle.p.pitch));
    means_cos[particle.which_cluster].pitch +=
        (particle.w * std::cos(particle.p.pitch));
    means_sin[particle.which_cluster].yaw += (particle.w * std::sin(particle.p.yaw));
    means_cos[particle.which_cluster].yaw += (particle.w * std::cos(particle.p.yaw));

    sums[particle.which_cluster] += particle.w;
  }
  for (int cluster = 0; cluster < k_clusters; cluster++) {
    if (sums[cluster] > 0.) {
      means[cluster].x /= sums[cluster];
      means[cluster].y /= sums[cluster];
      means[cluster].z /= sums[cluster];
      means[cluster].roll  = std::atan2(means_sin[cluster].roll / sums[cluster],
                                       means_cos[cluster].roll / sums[cluster]);
      means[cluster].pitch = std::atan2(means_sin[cluster].pitch / sums[cluster],
                                        means_cos[cluster].pitch / sums[cluster]);
      means[cluster].yaw   = std::atan2(means_sin[cluster].yaw / sums[cluster],
                                      means_cos[cluster].yaw / sums[cluster]);
    }
  }

  // ---- Weighted standard deviation
  std::vector<pose>  stds(k_clusters, pose(0., 0., 0., 0., 0., 0.));
  std::vector<float> non_zero(k_clusters, 0.);
  for (const auto& particle : particles) {
    float w_x = particle.w * ((particle.p.x - means[particle.which_cluster].x) *
                              (particle.p.x - means[particle.which_cluster].x));
    float w_y = particle.w * ((particle.p.y - means[particle.which_cluster].y) *
                              (particle.p.y - means[particle.which_cluster].y));
    float w_z = particle.w * ((particle.p.x - means[particle.which_cluster].x) *
                              (particle.p.x - means[particle.which_cluster].z));
    float w_roll =
        particle.w * ((particle.p.roll - means[particle.which_cluster].roll) *
                      (particle.p.roll - means[particle.which_cluster].roll));
    float w_pitch =
        particle.w * ((particle.p.pitch - means[particle.which_cluster].pitch) *
                      (particle.p.pitch - means[particle.which_cluster].pitch));
    float w_yaw =
        particle.w * ((particle.p.yaw - means[particle.which_cluster].yaw) *
                      (particle.p.yaw - means[particle.which_cluster].yaw));

    stds[particle.which_cluster].x += w_x;
    stds[particle.which_cluster].y += w_y;
    stds[particle.which_cluster].z += w_z;
    stds[particle.which_cluster].roll += w_roll;
    stds[particle.which_cluster].pitch += w_pitch;
    stds[particle.which_cluster].yaw += w_yaw;

    non_zero[particle.which_cluster] =
        (particle.w > 0.)
            ? (non_zero[particle.which_cluster] + static_cast<float>(1.))
            : non_zero[particle.which_cluster];
  }
  for (int cluster = 0; cluster < k_clusters; cluster++) {
    if (sums[cluster] > 0. && non_zero[cluster] > 1.) {
      stds[cluster].x =
          std::sqrt(stds[cluster].x /
                    ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));
      stds[cluster].y =
          std::sqrt(stds[cluster].y /
                    ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));
      stds[cluster].z =
          std::sqrt(stds[cluster].z /
                    ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));
      stds[cluster].roll =
          std::sqrt(stds[cluster].roll /
                    ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));
      stds[cluster].pitch =
          std::sqrt(stds[cluster].pitch /
                    ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));
      stds[cluster].yaw =
          std::sqrt(stds[cluster].yaw /
                    ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));

      Gaussian<pose, pose> m_gaussian(means[cluster], stds[cluster]);
      gauss_map[cluster] = m_gaussian;
    }
  }
}

void PF::scanMatch(const std::vector<ImageFeature>&     features,
                   OccupancyMap*                        grid_map,
                   std::map<int, Gaussian<pose, pose>>& gauss_map,
                   std::vector<float>&                  ws)
{
  float normalizer_icp =
      static_cast<float>(1.) / (sigma_feature_matching * std::sqrt(M_2PI));

  std::map<int, TF> tfs;
  // -------------------------------------------------------------------------------
  // ------- 3D scan matching using low level features
  // -------------------------------------------------------------------------------
  // - Set target point clouds (common to all clusters)
  icp->setInputTarget(grid_map);
  icp->setInputSource(features);
  // - Perform scan matching for each cluster
  bool                 valid_it = true;
  std::map<int, float> cluster_ws;
  for (auto& it : gauss_map) {
    // Convert cluster pose to [R|t]
    std::array<float, 3> trans = {
        it.second.mean.x, it.second.mean.y, it.second.mean.z};
    std::array<float, 9> Rot{};
    it.second.mean.toRotMatrix(Rot);

    // --------------- Perform scan matching ---------------------
    std::vector<ImageFeature> aligned;
    float                     rms_error;
    TF                        m_tf;
    TF                        final_tf;
    TF                        original_tf(Rot, trans);
    // - First guess: each particle drawn by odometry motion model
    // - Only use scan match if it does no fail
    if (icp->align(original_tf, rms_error, aligned)) {
      // ---------------------------------------------------------------------------
      // ------------ Get homogeneous transformation result
      // ---------------------------------------------------------------------------
      icp->getTransform(m_tf);

      final_tf = original_tf.inverse() * m_tf;

      // ---------------------------------------------------------------------------
      // ----------- Compute scan match weights
      // ---------------------------------------------------------------------------
      // - Get the correspondences errors both spatial and for the descriptors
      std::vector<float> serror;
      icp->getErrors(serror);

      // - Prevent single correspondence - standard deviation = 0
      if (serror.size() <= 1) {
        final_tf.R   = std::array<float, 9>{1., 0., 0., 0., 1., 0., 0., 0., 1.};
        final_tf.t   = std::array<float, 3>{0., 0., 0.};
        ws[it.first] = 0.;
        continue;
      }

      // - Compute weight
      float w = 0.;
      for (float i : serror) {
        w += static_cast<float>(normalizer_icp *
                                exp(-1. / sigma_feature_matching * i));
      }

      cluster_ws[it.first] = w;
      valid_it             = true;
    } else {
      final_tf.R = std::array<float, 9>{1., 0., 0., 0., 1., 0., 0., 0., 1.};
      final_tf.t = std::array<float, 3>{0., 0., 0.};

      ws[it.first] = 0.;
    }

    // Get delta transform
    tfs[it.first] = final_tf;
  }

  // -------------------------------------------------------------------------------
  // ------- Apply ICP result to the clusters and update particles weights
  // -------------------------------------------------------------------------------
  if (valid_it) {
    for (auto& particle : particles) {
      // - ICP result application
      TF m_tf = tfs[particle.which_cluster];

      // Convert cluster pose to [R|t]
      std::array<float, 3> trans = {particle.p.x, particle.p.y, particle.p.z};
      std::array<float, 9> Rot{};
      particle.p.toRotMatrix(Rot);

      TF particle_tf(Rot, trans);

      particle_tf = particle_tf * m_tf;
      particle.p  = pose(particle_tf.R, particle_tf.t);
      particle.p.normalize();

      // Update particle weight
      ws[particle.id] = cluster_ws[particle.which_cluster];
    }
  }
}

void PF::normalizeWeights()
{
  if (w_sum > 0.) {
    for (auto& particle : particles) particle.w /= w_sum;
  } else {
    for (auto& particle : particles)
      particle.w = static_cast<float>(1.) / static_cast<float>(particles.size());
  }
}

void PF::resample()
{
  float    cweight = 0.;
  uint32_t n       = particles.size();

  // - Compute the cumulative weights
  for (const auto& p : particles) cweight += p.w;
  // - Compute the interval
  float interval = cweight / n;
  // - Compute the initial target weight
  auto target = static_cast<float>(interval * ::drand48());

  // - Compute the resampled indexes
  cweight = 0.;
  std::vector<uint32_t> indexes(n);
  n          = 0.;
  uint32_t i = 0;

  for (const auto& p : particles) {
    cweight += p.w;
    while (cweight > target) {
      indexes[n++] = i;
      target += interval;
    }

    i++;
  }

  // - Update particle set
  for (size_t j = 0; j < indexes.size(); j++) {
    particles[j].pp            = particles[indexes[j]].pp;
    particles[j].ptf           = particles[indexes[j]].ptf;
    particles[j].p             = particles[indexes[j]].p;
    particles[j].tf            = particles[indexes[j]].tf;
    particles[j].w             = particles[indexes[j]].w;
    particles[j].which_cluster = particles[indexes[j]].which_cluster;
  }
}

} // namespace vineslam
