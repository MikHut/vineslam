#include "pf.hpp"

#include <cmath>

namespace vineslam
{

PF::PF(const std::string& config_path,
       const pose&        initial_pose,
       const int&         m_n_particles)
    : config_path(config_path)
{
  // Read input parameters
  YAML::Node config = YAML::LoadFile(config_path);
  cam_pitch         = config["camera_info"]["cam_pitch"].as<float>() * DEGREE_TO_RAD;
  srr               = config["pf"]["srr"].as<float>();
  str               = config["pf"]["str"].as<float>();
  stt               = config["pf"]["stt"].as<float>();
  srt               = config["pf"]["srt"].as<float>();
  sigma_xy          = config["pf"]["sigma_xy"].as<float>();
  sigma_z           = config["pf"]["sigma_z"].as<float>();
  sigma_roll        = config["pf"]["sigma_roll"].as<float>() * DEGREE_TO_RAD;
  sigma_pitch       = config["pf"]["sigma_pitch"].as<float>() * DEGREE_TO_RAD;
  sigma_yaw         = config["pf"]["sigma_yaw"].as<float>() * DEGREE_TO_RAD;
  sigma_landmark_matching = config["pf"]["sigma_landmark_matching"].as<float>();
  sigma_feature_matching  = config["pf"]["sigma_feature_matching"].as<float>();
  sigma_corner_matching   = config["pf"]["sigma_corner_matching"].as<float>();
  sigma_ground_rp = config["pf"]["sigma_ground_rp"].as<float>() * DEGREE_TO_RAD;
  sigma_gps       = config["pf"]["sigma_gps"].as<float>();
  k_clusters      = config["pf"]["k_clusters"].as<int>();
  k_iterations    = config["pf"]["k_iterations"].as<int>();
  n_particles     = m_n_particles;

  // Initialize normal distributions
  particles.resize(n_particles);

  // Initialize all particles
  for (size_t i = 0; i < particles.size(); i++) {
    // Calculate the initial pose for each particle considering
    // - the input initial pose
    // - a sample distribution to spread the particles
    pose m_pose(sampleGaussian(sigma_xy, i),
                sampleGaussian(sigma_xy, i),
                sampleGaussian(sigma_z, i),
                sampleGaussian(sigma_roll, i),
                sampleGaussian(sigma_pitch, i),
                sampleGaussian(sigma_yaw, i));
    // Compute initial weight of each particle
    float weight = 1.;
    // Insert the particle into the particles array
    particles[i] = Particle(i, m_pose, weight);
  }

  // Set last iteration vars
  p_odom = initial_pose;
}

void PF::motionModel(const pose& odom)
{
  // Compute odometry increment
  pose odom_inc = odom - p_odom;

  // Compute the relative pose given by the odometry motion model
  float dt_trans = odom_inc.norm2D();
  float dt_rot_a =
      ((odom_inc.x != 0) || (odom_inc.y != 0))
          ? normalizeAngle(std::atan2(odom_inc.y, odom_inc.x) - p_odom.yaw)
          : static_cast<float>(0.);
  float dt_rot_b = normalizeAngle(odom_inc.yaw - dt_rot_a);

  // Motion sample standard deviations
  float comp = (odom_inc.x < 0)
                   ? M_PI
                   : static_cast<float>(
                         0.); // PREVENT ERROR WHEN ROBOT IS MOVING BACKWARDS :-)
  float s_rot_a_draw = srr * (std::fabs(dt_rot_a) - comp) + srt * dt_trans;
  float s_rot_b_draw = srr * (std::fabs(dt_rot_b) - comp) + srt * dt_trans;
  float s_trans_draw =
      stt * dt_trans + str * (std::fabs(dt_rot_a) + std::fabs(dt_rot_b) - 2 * comp);

  // Apply the motion model to all particles
  for (auto& particle : particles) {
    // Sample the normal distribution functions
    float s_rot_a = dt_rot_a + sampleGaussian(s_rot_a_draw);
    float s_rot_b = dt_rot_b + sampleGaussian(s_rot_b_draw);
    float s_trans = dt_trans + sampleGaussian(s_trans_draw);

    // Compute the relative pose considering the samples
    pose dt_pose;
    dt_pose.x = s_trans * std::cos(normalizeAngle(particle.p.yaw + s_rot_a));
    // sampleGaussian(0.001);
    dt_pose.y = s_trans * std::sin(normalizeAngle(particle.p.yaw + s_rot_a));
    // sampleGaussian(0.001);
    dt_pose.z     = -sampleGaussian(sigma_z);
    dt_pose.roll  = -sampleGaussian(sigma_roll);
    dt_pose.pitch = -sampleGaussian(sigma_pitch);
    dt_pose.yaw   = s_rot_a + s_rot_b; //+ sampleGaussian(0.0005);

    // Innovate particles using the odometry motion model
    particle.p.x += dt_pose.x;
    particle.p.y += dt_pose.y;
    particle.p.z += dt_pose.z;
    particle.p.roll += dt_pose.roll;
    particle.p.pitch += dt_pose.pitch;
    particle.p.yaw += dt_pose.yaw;
  }

  p_odom = odom;
}

void PF::update(const int&                          xmin,
                const int&                          xmax,
                const std::vector<SemanticFeature>& landmarks,
                const std::vector<Corner>&          corners,
                const Plane&                        ground_plane,
                const pose&                         gps_pose,
                OccupancyMap                        grid_map)
{
  // -------------------------------------------------------------------------------
  // --- 3D ground plane [roll, pitch, z] estimation - only done once
  // -------------------------------------------------------------------------------
  pose                   pose_from_ground;
  Gaussian<float, float> ground_plane_gauss{};
  if (ground_plane.points.size() > 1) {
    // - Compute rotation matrix that transform the normal vector into a vector
    // perpendicular to the plane z = 0
    // ---- in other words, the rotation matrix that aligns the ground plane with
    // the plane z = 0
    // ---- this matrix will encode the absolute roll and pitch of the robot
    std::array<float, 9> R{};
    vector3D             m_normal = ground_plane.normal;
    float norm = std::sqrt(m_normal.x * m_normal.x + m_normal.y * m_normal.y);
    R[0]       = +m_normal.y / norm;
    R[1]       = -m_normal.x / norm;
    R[2]       = 0.;
    R[3]       = (m_normal.x * m_normal.z) / norm;
    R[4]       = (m_normal.y * m_normal.z) / norm;
    R[5]       = -norm;
    R[6]       = m_normal.x;
    R[7]       = m_normal.y;
    R[8]       = m_normal.z;

    // - Compute the local ground plane altimetry as well as a gaussian representing
    // the validity of the ground plane estimation
    // ---- the higher the standard deviation of the z distances, the lower the
    // precision of the estimation, since both planes should be paralel
    // Mean
    float              z_mean = 0.;
    std::vector<float> zs;
    std::vector<point> z_null_plane;
    for (const auto& pt : ground_plane.points) {
      float z = (pt.x * R[6] + pt.y * R[7] + pt.z * R[8]);
      z_mean += z;
      zs.push_back(z);
    }
    z_mean /= static_cast<float>(zs.size());

    // Standard deviation
    float z_std = 0.;
    for (const auto& m_z : zs) z_std += (m_z - z_mean) * (m_z - z_mean);
    z_std = std::sqrt(z_std / static_cast<float>(zs.size()));
    // Save as gaussian
    ground_plane_gauss = Gaussian<float, float>(z_mean, z_std);

    // - Compute the componenets of interest and save as pose
    std::array<float, 3> altimetry = {0., 0., z_mean};
    pose_from_ground               = pose(R, altimetry);
  } else {
    ground_plane_gauss = Gaussian<float, float>(0., 0.);
  }

  // Compute static vars to use in the PF loop
  float normalizer_landmark =
      static_cast<float>(1.) / (sigma_landmark_matching * std::sqrt(M_2PI));
  float normalizer_corner =
      static_cast<float>(1.) / (sigma_feature_matching * std::sqrt(M_2PI));
  float normalizer_ground_rp =
      static_cast<float>(1.) / (sigma_ground_rp * std::sqrt(M_2PI));
  float normalizer_gps = static_cast<float>(1.) / (sigma_gps * std::sqrt(M_2PI));
  // ----- Check if we want or not to use GPS
  bool use_gps =
      !(gps_pose.x == -0. && gps_pose.y == -0. && gps_pose.z == -0. &&
        gps_pose.roll == -0. && gps_pose.pitch == -0. && gps_pose.yaw == -0.);

  // Declare arrays to save the unnormalized weights
  std::vector<float> semantic_weights(n_particles);
  std::vector<float> corners_weights(n_particles);
  std::vector<float> ground_weights(n_particles);
  std::vector<float> gps_weights(n_particles);

  // Loop over all particles
  for (int i = xmin; i < xmax; i++) {
    Particle& particle = particles[i];
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
      // Convert landmark to the particle's referential frame
      point X;
      X.x = landmark.pos.x * Rot[0] + landmark.pos.y * Rot[1] +
            landmark.pos.z * Rot[2] + m_pose.x;
      X.y = landmark.pos.x * Rot[3] + landmark.pos.y * Rot[4] +
            landmark.pos.z * Rot[5] + m_pose.y;
      X.z = 0.;

      // Search for a correspondence in the current cell first
      float best_correspondence = std::numeric_limits<float>::max();
      bool  found               = false;
      for (const auto& m_landmark : grid_map(X.x, X.y).landmarks) {
        float dist_min = X.distanceXY(m_landmark.second.pos);

        if (dist_min < best_correspondence) {
          best_correspondence = dist_min;
          found               = true;
        }
      }

      // Only search in the adjacent cells if we do not find in the source cell
      if (!found) {
        std::vector<Cell> adjacents;
        grid_map.getAdjacent(X.x, X.y, 2, adjacents);
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

    // ------------------------------------------------------
    // --- 3D corner map fitting
    // ------------------------------------------------------
    Rot = {}; // clear rotation matrix to set for 3D estimation
    particle.p.toRotMatrix(Rot);
    std::vector<float> dcornervec;
    for (const auto& corner : corners) {
      // Convert landmark to the particle's referential frame
      point X;
      X.x = corner.pos.x * Rot[0] + corner.pos.y * Rot[1] + corner.pos.z * Rot[2] +
            particle.p.x;
      X.y = corner.pos.x * Rot[3] + corner.pos.y * Rot[4] + corner.pos.z * Rot[5] +
            particle.p.y;
      X.z = corner.pos.x * Rot[6] + corner.pos.y * Rot[7] + corner.pos.z * Rot[8] +
            particle.p.z;

      // Search for a correspondence in the current cell first
      // TODO (André Aguiar): Changle float max to 0.02 (e.g.) when this is on GPU
      float best_correspondence = std::numeric_limits<float>::max();
      bool  found               = false;
      for (const auto& m_corner : grid_map(X.x, X.y).corner_features) {
        float dist_min = X.distance(m_corner.pos);

        if (dist_min < best_correspondence) {
          best_correspondence = dist_min;
          found               = true;
        }
      }

      // Only search in the adjacent cells if we do not find in the source cell
      if (!found) {
        std::vector<Cell> adjacents;
        grid_map.getAdjacent(X.x, X.y, 1, adjacents);
        for (const auto& m_cell : adjacents) {
          for (const auto& m_corner : m_cell.corner_features) {
            float dist_min = X.distance(m_corner.pos);
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
        dcornervec.push_back(best_correspondence);
    }

    // ------------------------------------------------------
    // --- Particle weight update
    // ------------------------------------------------------
    // - Semantic landmark matching [x, y, yaw] weight
    float w_landmarks = 0.;
    if (dlandmarkvec.size() <= 1) {
      w_landmarks = 0.;
    } else {
      for (const auto& dist : dlandmarkvec)
        w_landmarks +=
            (normalizer_landmark *
             static_cast<float>(std::exp(-1. / sigma_landmark_matching * dist)));
    }
    // - Corner feature matching [x, y, z, roll, pitch, yaw] weight
    float w_corners = 0.;
    if (dcornervec.size() <= 1) {
      w_corners = 0.;
    } else {

      for (const auto& dist : dcornervec) {
        w_corners +=
            (normalizer_corner *
             static_cast<float>(std::exp(-1. / sigma_corner_matching * dist)));
      }
    }
    // - Ground plane [roll, pitch] weight
    float w_ground = 0.;
    if (ground_plane_gauss.mean != 0. && ground_plane_gauss.stdev != 0.) {
      w_ground =
          (normalizer_ground_rp *
           static_cast<float>(
               std::exp(-1. / sigma_ground_rp *
                        std::fabs(particle.p.roll - pose_from_ground.roll)))) *
          (normalizer_ground_rp *
           static_cast<float>(
               std::exp(-1. / sigma_ground_rp *
                        std::fabs(particle.p.pitch - pose_from_ground.pitch))));
    }
    // - GPS [x, y] weight
    float w_gps = 0.;
    if (use_gps) {
      float dist = particle.p.distance(gps_pose);
      w_gps =
          (normalizer_gps * static_cast<float>(std::exp(-1. / sigma_gps * dist)));
    }

    // - Save each layer weight
    semantic_weights[particle.id] = w_landmarks;
    corners_weights[particle.id]  = w_corners;
    ground_weights[particle.id]   = w_ground;
    gps_weights[particle.id]      = w_gps;
  }

  // Multi-modal weights normalization
  float semantic_max =
      *std::max_element(semantic_weights.begin(), semantic_weights.end());
  float corners_max =
      *std::max_element(corners_weights.begin(), corners_weights.end());
  float ground_max = *std::max_element(ground_weights.begin(), ground_weights.end());
  float gps_max    = *std::max_element(gps_weights.begin(), gps_weights.end());
  for (int i = xmin; i < xmax; i++) {
    Particle& particle = particles[i];
    float     m_lw =
        (semantic_max > 0.) ? semantic_weights[particle.id] : static_cast<float>(1.);
    float m_cw =
        (corners_max > 0.) ? corners_weights[particle.id] : static_cast<float>(1.);
    float m_gw =
        (ground_max > 0.) ? ground_weights[particle.id] : static_cast<float>(1.);
    float m_gpsw =
        (gps_max > 0.) ? gps_weights[particle.id] : static_cast<float>(1.);

    particle.w = m_lw * m_cw * m_gw * m_gpsw;
    w_sum += particle.w;
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
    particles[j].p = particles[indexes[j]].p;
    particles[j].w = particles[indexes[j]].w;
  }
}

void PF::cluster(std::map<int, Gaussian<pose, pose>>& gauss_map)
{
  // -------------------------------------------------------------------------------
  // ------ (1) Initialize the clusters - kmean++
  // ------ (1) from: https://www.geeksforgeeks.org/ml-k-means-algorithm/
  // -------------------------------------------------------------------------------
  // -- Select first centroid randomly
  std::vector<pose> centroids;
  int               n = particles.size();
  srand(time(0)); // need to set the random seed
  centroids.push_back(particles[rand() % n].p);
  // -- Compute remaining k - 1 centroids
  for (int i = 0; i < k_clusters - 1; i++) {
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

  for (int i = 0; i < k_iterations; i++) {
    // -----------------------------------------------------------------------------
    // ------ (2) Assign the particles to clusters
    // -----------------------------------------------------------------------------
    for (auto& particle : particles) {
      float min_dist      = std::numeric_limits<float>::max();
      int   which_cluster = 0;
      for (int cluster = 0; cluster < k_clusters; cluster++) {
        float m_dist = particle.p.distance(centroids[cluster]);
        if (m_dist < min_dist) {
          min_dist      = m_dist;
          which_cluster = cluster;
        }
      }
      particle.which_cluster = which_cluster;
    }

    // -----------------------------------------------------------------------------
    // ------ (3) Re-compute the centroids
    // -----------------------------------------------------------------------------
    // TODO (André Aguiar): Consider the particles weight to compute the centroid
    std::vector<int> num_per_cluster(k_clusters, 0);
    for (const auto& particle : particles) {
      centroids[particle.which_cluster].x += particle.p.x;
      centroids[particle.which_cluster].y += particle.p.y;
      centroids[particle.which_cluster].z += particle.p.z;

      num_per_cluster[particle.which_cluster]++;
    }

    for (int j = 0; j < centroids.size(); j++) {
      if (num_per_cluster[j] == 0) {
        centroids[j] = pose(0., 0., 0., 0., 0., 0.);
        continue;
        ;
      } else {
        centroids[j].x /= num_per_cluster[j];
        centroids[j].y /= num_per_cluster[j];
        centroids[j].z /= num_per_cluster[j];
      }
    }
  }

  // -----------------------------------------------------------------------------
  // ------ (4) Compute Gaussian approximation of each cluster
  // -----------------------------------------------------------------------------
  for (int cluster = 0; cluster < k_clusters; cluster++) {
    pose centroid = centroids[cluster];
    if (centroid.x == 0. && centroid.y == 0. && centroid.z == 0.) {
      continue;
    } else {
      pose  stdev(0., 0., 0., 0., 0., 0.);
      float n_pts = 0;
      for (const auto& particle : particles) {
        if (particle.which_cluster == cluster) {
          stdev.x += ((particle.p.x - centroid.x) * (particle.p.x - centroid.x));
          stdev.y += ((particle.p.y - centroid.y) * (particle.p.y - centroid.y));
          stdev.z += ((particle.p.z - centroid.z) * (particle.p.z - centroid.z));

          n_pts = n_pts + static_cast<float>(1.);
        }
      }
      stdev.x = std::sqrt(stdev.x / n_pts);
      stdev.y = std::sqrt(stdev.y / n_pts);
      stdev.z = std::sqrt(stdev.z / n_pts);

      Gaussian<pose, pose> m_gaussian(centroid, stdev);
      gauss_map[cluster] = m_gaussian;
    }
  }
}

void PF::scanMatch() {}

} // namespace vineslam
