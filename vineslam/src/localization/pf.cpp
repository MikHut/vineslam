#include "../../include/vineslam/localization/pf.hpp"

namespace vineslam
{
PF::PF(const Parameters& params, const Pose& initial_pose) : params_(params)
{
  // - General parameters
  use_semantic_features_ = params.use_semantic_features_;
  use_lidar_features_ = params.use_lidar_features_;
  use_image_features_ = params.use_image_features_;
  use_gps_ = params.use_gps_;

  // Initialize and set ICP parameters
  icp_ = new ICP<ImageFeature>();
  icp_->setTolerance(1e-5);
  icp_->setMaxIterations(20);
  icp_->setRejectOutliersFlag(false);

  // Initialize thread pool
  thread_pool_ = new lama::ThreadPool;
  thread_pool_->init(NUM_THREADS);

  // Initialize profiler
  t_ = new Timer("Particle Filter");

  // Initialize normal distributions
  particles_.resize(params.number_particles_);

  // Initialize all particles
  for (size_t i = 0; i < particles_.size(); i++)
  {
    // Calculate the initial pose for each particle considering
    // - the input initial pose
    // - a sample distribution to spread the particles
    Pose m_pose = initial_pose + Pose(sampleGaussian(0.2), sampleGaussian(0.2), sampleGaussian(0.05),
                                      sampleGaussian(3 * DEGREE_TO_RAD), sampleGaussian(3 * DEGREE_TO_RAD),
                                      sampleGaussian(10 * DEGREE_TO_RAD));
    // Compute initial weight of each particle
    float weight = 1.;
    // Insert the particle into the particles array
    particles_[i] = Particle(i, m_pose, weight);
  }

  // Set filter settings
  sigma_landmark_matching_ = 0.01;
  sigma_feature_matching_ = 0.05;
  sigma_corner_matching_ = 0.1;
  sigma_planar_matching_ = 0.1;
  sigma_plane_matching_vector_ = 0.02;
  sigma_plane_matching_centroid_ = 0.10;
  sigma_gps_ = 0.08;
  number_clusters_ = 3;
}

void PF::motionModel(const Pose& odom_inc, const Pose& p_odom)
{
  float d_trans = odom_inc.norm3D();

  // Innovate particles
  for (auto& particle : particles_)
  {
    // Build pose noise transformation matrix
    Pose pose_noise(params_.sigma_xx_, params_.sigma_yy_, params_.sigma_zz_, params_.sigma_RR_, params_.sigma_PP_,
                    params_.sigma_YY_);

    std::array<float, 6> gaussian_noise{};
    for (float& i : gaussian_noise)
      i = d_trans * sampleGaussian(1.0);

    pose_noise.x_ *= gaussian_noise[0];  // xx
    pose_noise.y_ *= gaussian_noise[1];  // yy
    pose_noise.z_ *= gaussian_noise[2];  // zz
    pose_noise.R_ *= gaussian_noise[3];  // RR
    pose_noise.P_ *= gaussian_noise[4];  // PP
    pose_noise.Y_ *= gaussian_noise[5];  // YY

    std::array<float, 9> R_noise{};
    pose_noise.toRotMatrix(R_noise);
    Tf odom_noise_tf(R_noise, std::array<float, 3>{ pose_noise.x_, pose_noise.y_, pose_noise.z_ });

    // Built odom increment transformation matrix
    std::array<float, 9> R_inc{};
    odom_inc.toRotMatrix(R_inc);
    Tf odom_inc_tf(R_inc, std::array<float, 3>{ odom_inc.x_, odom_inc.y_, odom_inc.z_ });

    // Final pose increment
    Tf innovation_tf = odom_inc_tf * odom_noise_tf;

    // Save info for the next iteration
    particle.pp_ = particle.p_;
    particle.ptf_ = particle.tf_;

    // Apply transformation
    particle.tf_ = particle.ptf_ * innovation_tf;
    particle.p_ = Pose(particle.tf_.R_array_, particle.tf_.t_array_);
  }
}

void PF::update(const std::vector<SemanticFeature>& landmarks, const std::vector<Corner>& corners,
                const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes, const SemiPlane& ground_plane,
                const std::vector<ImageFeature>& surf_features, const Pose& gps_pose, OccupancyMap* grid_map)
{
  std::vector<float> semantic_weights(params_.number_particles_, 0.);
  std::vector<float> corner_weights(params_.number_particles_, 0.);
  std::vector<float> planar_weights(params_.number_particles_, 0.);
  std::vector<float> planes_weights(params_.number_particles_, 0.);
  std::vector<float> ground_weights(params_.number_particles_, 0.);
  std::vector<float> surf_weights(params_.number_particles_, 0.);
  std::vector<float> gps_weights(params_.number_particles_, 0.);

  logs_ = "\n";

  auto before = std::chrono::high_resolution_clock::now();
  if (use_semantic_features_)
    highLevel(landmarks, grid_map, semantic_weights);
  auto after = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float, std::milli> duration = after - before;
  logs_ += "Time elapsed on PF - high-level features (msecs): " + std::to_string(duration.count()) + "\n";

  if (use_lidar_features_)
  {
    before = std::chrono::high_resolution_clock::now();
    t_->tick("pf::corners()");
    mediumLevelCorners(corners, grid_map, corner_weights);
    t_->tock();
    after = std::chrono::high_resolution_clock::now();
    duration = after - before;
    logs_ += "Time elapsed on PF - corner features (msecs): " + std::to_string(duration.count()) + " (" +
             std::to_string(corners.size()) + ")\n";

    before = std::chrono::high_resolution_clock::now();
    t_->tick("pf::planars()");
    mediumLevelPlanars(planars, grid_map, planar_weights);
    t_->tock();
    after = std::chrono::high_resolution_clock::now();
    duration = after - before;
    logs_ += "Time elapsed on PF - planar features (msecs): " + std::to_string(duration.count()) + " (" +
             std::to_string(planars.size()) + ")\n";

    before = std::chrono::high_resolution_clock::now();
    t_->tick("pf::planes()");
    mediumLevelPlanes({ ground_plane }, grid_map, ground_weights);
    mediumLevelPlanes(planes, grid_map, planes_weights);
    t_->tock();
    after = std::chrono::high_resolution_clock::now();
    duration = after - before;
    logs_ += "Time elapsed on PF - ground plane (msecs): " + std::to_string(duration.count()) + "\n";
    before = std::chrono::high_resolution_clock::now();
  }

  if (use_image_features_)
    lowLevel(surf_features, grid_map, surf_weights);
  after = std::chrono::high_resolution_clock::now();
  duration = after - before;
  logs_ += "Time elapsed on PF - icp (msecs): " + std::to_string(duration.count()) + "\n";
  before = std::chrono::high_resolution_clock::now();
  if (use_gps_)
    gps(gps_pose, gps_weights);
  after = std::chrono::high_resolution_clock::now();
  duration = after - before;
  logs_ += "Time elapsed on PF - gps (msecs): " + std::to_string(duration.count()) + "\n";

  // Multi-modal weights normalization
  float semantic_max = *std::max_element(semantic_weights.begin(), semantic_weights.end());
  float corners_max = *std::max_element(corner_weights.begin(), corner_weights.end());
  float planars_max = *std::max_element(planar_weights.begin(), planar_weights.end());
  float planes_max = *std::max_element(planes_weights.begin(), planes_weights.end());
  float ground_max = *std::max_element(ground_weights.begin(), ground_weights.end());
  float surf_max = *std::max_element(surf_weights.begin(), surf_weights.end());
  float gps_max = *std::max_element(gps_weights.begin(), gps_weights.end());
  for (auto& particle : particles_)
  {
    float m_lw = (semantic_max > 0.) ? semantic_weights[particle.id_] : static_cast<float>(1.);
    float m_cw = (corners_max > 0.) ? corner_weights[particle.id_] : static_cast<float>(1.);
    float m_rw = (planars_max > 0.) ? planar_weights[particle.id_] : static_cast<float>(1.);
    float m_pw = (planes_max > 0.) ? planes_weights[particle.id_] : static_cast<float>(1.);
    float m_gw = (ground_max > 0.) ? ground_weights[particle.id_] : static_cast<float>(1.);
    float m_sw = (surf_max > 0.) ? surf_weights[particle.id_] : static_cast<float>(1.);
    float m_gpsw = (gps_max > 0.) ? gps_weights[particle.id_] : static_cast<float>(1.);

    particle.w_ = m_lw * m_cw * m_rw * m_pw * m_gw * m_sw * m_gpsw;

    w_sum_ += particle.w_;
  }

  t_->getLog();
  t_->clearLog();
}

void PF::gps(const Pose& gps_pose, std::vector<float>& ws)
{
  float normalizer_gps = static_cast<float>(1.) / (sigma_gps_ * std::sqrt(M_2PI));

  for (const auto& particle : particles_)
  {
    // - GPS [x, y] weight
    float w_gps;
    float dist = particle.p_.distance(gps_pose);
    w_gps = (normalizer_gps * static_cast<float>(std::exp(-1. / sigma_gps_ * dist)));

    ws[particle.id_] = w_gps;
  }
}

void PF::highLevel(const std::vector<SemanticFeature>& landmarks, OccupancyMap* grid_map, std::vector<float>& ws)
{
  float normalizer_landmark = static_cast<float>(1.) / (sigma_landmark_matching_ * std::sqrt(M_2PI));

  // Loop over all particles
  for (const auto& particle : particles_)
  {
    // Convert particle orientation to rotation matrix
    Pose m_pose = particle.p_;
    m_pose.R_ = 0.;
    m_pose.P_ = 0.;
    m_pose.z_ = 0.;
    std::array<float, 9> Rot{};
    m_pose.toRotMatrix(Rot);

    // ------------------------------------------------------
    // --- 2D semantic feature map fitting
    // ------------------------------------------------------
    std::vector<float> dlandmarkvec;
    for (const auto& landmark : landmarks)
    {
      // Convert landmark to the maps's referential frame
      Point X;
      X.x_ = landmark.pos_.x_ * Rot[0] + landmark.pos_.y_ * Rot[1] + landmark.pos_.z_ * Rot[2] + m_pose.x_;
      X.y_ = landmark.pos_.x_ * Rot[3] + landmark.pos_.y_ * Rot[4] + landmark.pos_.z_ * Rot[5] + m_pose.y_;
      X.z_ = 0.;

      // Search for a correspondence in the current cell first
      float best_correspondence = std::numeric_limits<float>::max();
      bool found = false;
      for (const auto& m_landmark : (*grid_map)(X.x_, X.y_, 0).landmarks_)
      {
        float dist_min = X.distanceXY(m_landmark.second.pos_);

        if (dist_min < best_correspondence)
        {
          best_correspondence = dist_min;
          found = true;
        }
      }

      // Only search in the adjacent cells if we do not find in the source cell
      if (!found)
      {
        std::vector<Cell> adjacents;
        grid_map->getAdjacent(X.x_, X.y_, 0, 2, adjacents);
        for (const auto& m_cell : adjacents)
        {
          for (const auto& m_landmark : m_cell.landmarks_)
          {
            float dist_min = X.distanceXY(m_landmark.second.pos_);
            if (dist_min < best_correspondence)
            {
              best_correspondence = dist_min;
              found = true;
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
    if (dlandmarkvec.size() <= 1)
    {
      w_landmarks = 0.;
    }
    else
    {
      for (const auto& dist : dlandmarkvec)
        w_landmarks += (normalizer_landmark * static_cast<float>(std::exp(-1. / sigma_landmark_matching_ * dist)));
    }

    ws[particle.id_] = w_landmarks;
  }
}

void PF::mediumLevelCorners(const std::vector<Corner>& corners, OccupancyMap* grid_map, std::vector<float>& ws)
{
  float normalizer_corner = static_cast<float>(1.) / (sigma_corner_matching_ * std::sqrt(M_2PI));

  // Loop over all particles
  //  for (const auto& particle : particles_)
  for (uint32_t i = 0; i < particles_.size(); ++i)
  {
#if NUM_THREADS > 1
    thread_pool_->enqueue([this, corners, grid_map, normalizer_corner, &ws, i]() {
#endif
      // ------------------------------------------------------
      // --- 3D corner map fitting
      // ------------------------------------------------------
      float w_corners = 0;
      std::vector<float> dcornervec;
      for (const auto& corner : corners)
      {
        // Convert feature to the map's referential frame
        Point X = corner.pos_ * particles_[i].tf_;

        std::vector<Corner> m_corners = (*grid_map)(X.x_, X.y_, X.z_).corner_features_;

        // Search for a correspondence in the current cell first
        float best_correspondence = 0.5;
        bool found = false;
        for (const auto& m_corner : m_corners)
        {
          float dist_sq = ((X.x_ - m_corner.pos_.x_) * (X.x_ - m_corner.pos_.x_) +
                           (X.y_ - m_corner.pos_.y_) * (X.y_ - m_corner.pos_.y_) +
                           (X.z_ - m_corner.pos_.z_) * (X.z_ - m_corner.pos_.z_));

          if (dist_sq < best_correspondence)
          {
            best_correspondence = dist_sq;
            found = true;
          }
        }

        // Save distance if a correspondence was found
        if (found)
          w_corners +=
              (normalizer_corner * static_cast<float>(std::exp(-1. / sigma_corner_matching_ * best_correspondence)));
      }

      ws[particles_[i].id_] = w_corners;
#if NUM_THREADS > 1
    });
#endif
  }

#if NUM_THREADS > 1
  thread_pool_->wait();
#endif
}

void PF::mediumLevelPlanars(const std::vector<Planar>& planars, OccupancyMap* grid_map, std::vector<float>& ws)
{
  float normalizer_planar = static_cast<float>(1.) / (sigma_planar_matching_ * std::sqrt(M_2PI));

  // Loop over all particles
  //  for (const auto& particle : particles_)
  for (uint32_t i = 0; i < particles_.size(); ++i)
  {
#if NUM_THREADS > 1
    thread_pool_->enqueue([this, planars, grid_map, normalizer_planar, &ws, i]() {
#endif
      // ------------------------------------------------------
      // --- 3D planar map fitting
      // ------------------------------------------------------
      float w_planars = 0.;
      for (const auto& planar : planars)
      {
        // Convert feature to the map's referential frame
        Point X = planar.pos_ * particles_[i].tf_;

        std::vector<Planar> m_planars = (*grid_map)(X.x_, X.y_, X.z_).planar_features_;

        // Search for a correspondence in the current cell first
        float best_correspondence = 0.5;
        bool found = false;
        for (const auto& m_planar : m_planars)
        {
          float dist_sq = ((X.x_ - m_planar.pos_.x_) * (X.x_ - m_planar.pos_.x_) +
                           (X.y_ - m_planar.pos_.y_) * (X.y_ - m_planar.pos_.y_) +
                           (X.z_ - m_planar.pos_.z_) * (X.z_ - m_planar.pos_.z_));

          if (dist_sq < best_correspondence)
          {
            best_correspondence = dist_sq;
            found = true;
          }
        }

        // Save distance if a correspondence was found
        if (found)
          w_planars +=
              (normalizer_planar * static_cast<float>(std::exp((-1. / sigma_planar_matching_) * best_correspondence)));
      }

      ws[particles_[i].id_] = w_planars;
#if NUM_THREADS > 1
    });
#endif
  }

#if NUM_THREADS > 1
  thread_pool_->wait();
#endif
}

void PF::mediumLevelPlanes(const std::vector<SemiPlane>& planes, OccupancyMap* grid_map, std::vector<float>& ws)
{
  float normalizer_plane_vector = static_cast<float>(1.) / (sigma_plane_matching_vector_ * std::sqrt(M_2PI));
  float normalizer_plane_centroid = static_cast<float>(1.) / (sigma_plane_matching_centroid_ * std::sqrt(M_2PI));

  // Loop over all particles
  //  for (const auto& particle : particles_)
  for (uint32_t i = 0; i < particles_.size(); ++i)
  {
#if NUM_THREADS > 1
    thread_pool_->enqueue([this, planes, grid_map, normalizer_plane_vector, normalizer_plane_centroid, &ws, i]() {
#endif
      float w_planes = 0.;
      // ----------------------------------------------------------------------------
      // ------ Search for correspondences between local planes and global planes
      // ------ Three stage process:
      // ------  * (A) Check semi-plane overlap
      // ------  * (B) Compare planes normals
      // ------  *  If (B), then check (C) plane to plane distance
      // ----------------------------------------------------------------------------

      // Define correspondence thresholds
      float v_dist = 0.2;   // max vector displacement for all the components
      float sp_dist = 0.2;  // max distance from source plane centroid to target plane
      float area_th = 2.0;  // minimum overlapping area between semiplanes

      // Correspondence result
      float correspondence_vec;
      float correspondence_centroid;

      for (const auto& plane : planes)
      {
        if (plane.points_.empty())
        {
          continue;
        }

        // Initialize correspondence deltas
        float vec_disp = v_dist;
        float point2plane = sp_dist;
        float ov_area = area_th;

        // Convert local plane to maps' referential frame
        SemiPlane l_plane = plane;
        for (auto& point : l_plane.points_)
        {
          point = point * particles_[i].tf_;  // Convert plane points
        }
        for (auto& point : l_plane.extremas_)
        {
          point = point * particles_[i].tf_;  // Convert plane boundaries
        }
        l_plane.centroid_ = l_plane.centroid_ * particles_[i].tf_;                        // Convert the centroid
        estimateNormal(l_plane.points_, l_plane.a_, l_plane.b_, l_plane.c_, l_plane.d_);  // Convert plane normal

        bool found = false;
        for (auto& g_plane : grid_map->planes_)
        {
          if (g_plane.points_.empty())
          {
            continue;
          }

          // --------------------------------
          // (A) - Check semi-plane overlap
          // --------------------------------

          // First project the global and local plane extremas to the global plane reference frame
          Tf ref_frame = g_plane.local_ref_.inverse();
          SemiPlane gg_plane;
          SemiPlane lg_plane;
          for (const auto& extrema : g_plane.extremas_)
          {
            Point p = extrema * ref_frame;
            p.z_ = 0;
            gg_plane.extremas_.push_back(p);
          }
          for (const auto& extrema : l_plane.extremas_)
          {
            Point p = extrema * ref_frame;
            p.z_ = 0;
            lg_plane.extremas_.push_back(p);
          }

          // Now, check for transformed polygon intersections
          SemiPlane isct;
          polygonIntersection(gg_plane, lg_plane, isct.extremas_);

          // Compute the intersection semi plane area
          isct.setArea();

          if (isct.area_ > ov_area)
          {
            // --------------------------------
            // (B) - Compare plane normals
            // --------------------------------

            Vec u(l_plane.a_, l_plane.b_, l_plane.c_);
            Vec v(g_plane.a_, g_plane.b_, g_plane.c_);

            float D = ((u - v).norm3D() < (u + v).norm3D()) ? (u - v).norm3D() : (u + v).norm3D();

            // Check if normal vectors match
            if (D < vec_disp)
            {
              // --------------------------------
              // (C) - Compute local plane centroid distance to global plane
              // --------------------------------
              float l_point2plane = g_plane.point2Plane(l_plane.centroid_);
              if (l_point2plane < point2plane)
              {
                // We found a correspondence, so, we must save the correspondence deltas
                vec_disp = D;
                point2plane = l_point2plane;
                ov_area = isct.area_;

                // Save correspondence errors
                correspondence_vec = D;
                correspondence_centroid = l_point2plane;

                // Set correspondence flag
                found = true;
              }
            }
          }
        }

        float vv = 0, cc = 0;
        if (found)
        {
          vv = ((normalizer_plane_vector *
                 static_cast<float>(std::exp((-1. / sigma_plane_matching_vector_) * correspondence_vec))));
          cc = ((normalizer_plane_centroid *
                 static_cast<float>(std::exp((-1. / sigma_plane_matching_centroid_) * correspondence_centroid))));
          w_planes +=
              ((normalizer_plane_vector *
                static_cast<float>(std::exp((-1. / sigma_plane_matching_vector_) * correspondence_vec))) *
               (normalizer_plane_centroid *
                static_cast<float>(std::exp((-1. / sigma_plane_matching_centroid_) * correspondence_centroid))));
        }
      }

      ws[particles_[i].id_] = w_planes;
#if NUM_THREADS > 1
    });
#endif
  }

#if NUM_THREADS > 1
  thread_pool_->wait();
#endif
}

void PF::lowLevel(const std::vector<ImageFeature>& surf_features, OccupancyMap* grid_map, std::vector<float>& ws)
{
  // ------------------------------------------------------------------------------
  // ---------------- Cluster particles
  // ------------------------------------------------------------------------------
  std::map<int, Gaussian<Pose, Pose>> gauss_map;
  cluster(gauss_map);

  // ------------------------------------------------------------------------------
  // ---------------- Scan match
  // ------------------------------------------------------------------------------
  scanMatch(surf_features, grid_map, gauss_map, ws);
}

void PF::cluster(std::map<int, Gaussian<Pose, Pose>>& gauss_map)
{
  // -------------------------------------------------------------------------------
  // ------ (1) Initialize the clusters - kmean++
  // ------ (1) from: https://www.geeksforgeeks.org/ml-k-means-algorithm/
  // -------------------------------------------------------------------------------
  // -- Select first centroid randomly
  std::vector<Pose> centroids;
  std::vector<Pose> m_centroids;
  int n = particles_.size();
  srand(time(nullptr));  // need to set the random seed
  centroids.push_back(particles_[rand() % n].p_);
  // -- Compute remaining k - 1 centroids
  for (int i = 1; i < number_clusters_; i++)
  {
    std::vector<float> min_dists;
    for (const auto& particle : particles_)
    {
      // Find minimum distance to already computed centroid
      float min_dist = std::numeric_limits<float>::max();
      for (auto& centroid : centroids)
      {
        float m_dist = particle.p_.distance(centroid);
        if (m_dist < min_dist)
        {
          min_dist = m_dist;
        }
      }
      min_dists.push_back(min_dist);
    }
    // Select data point with maximum distance as our next centroid
    auto it = std::max_element(min_dists.begin(), min_dists.end());
    int idx = std::distance(min_dists.begin(), it);
    centroids.push_back(particles_[idx].p_);
  }
  // -- Initialize equal-sized cluster
  std::vector<std::map<int, float>> heap(number_clusters_);
  for (int i = 0; i < number_clusters_; i++)
  {
    for (const auto& particle : particles_)
    {
      float dist = centroids[i].distance(particle.p_);
      heap[i][particle.id_] = dist;
    }
  }
  std::vector<int> num_per_cluster(number_clusters_, 0);
  int max_num = static_cast<int>(static_cast<float>(params_.number_particles_) / static_cast<float>(number_clusters_));
  for (auto& particle : particles_)
  {
    float min_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < number_clusters_; i++)
    {
      float dist = heap[i][particle.id_];
      if (dist < min_dist && num_per_cluster[i] < max_num)
      {
        particle.which_cluster_ = i;
        min_dist = dist;
      }
    }
    num_per_cluster[particle.which_cluster_]++;
  }

  int swaps = 0;
  int k_iterations = 20;
  for (int i = 0; i < k_iterations; i++)
  {
    // -----------------------------------------------------------------------------
    // ------ (2) Re-compute the centroids
    // -----------------------------------------------------------------------------
    num_per_cluster = std::vector<int>(number_clusters_, 0);
    for (const auto& particle : particles_)
    {
      centroids[particle.which_cluster_].x_ += particle.p_.x_;
      centroids[particle.which_cluster_].y_ += particle.p_.y_;
      centroids[particle.which_cluster_].z_ += particle.p_.z_;

      num_per_cluster[particle.which_cluster_]++;
    }

    for (int j = 0; j < number_clusters_; j++)
    {
      if (num_per_cluster[j] == 0)
      {
        centroids[j] = Pose(0., 0., 0., 0., 0., 0.);
        continue;
      }
      else
      {
        centroids[j].x_ /= num_per_cluster[j];
        centroids[j].y_ /= num_per_cluster[j];
        centroids[j].z_ /= num_per_cluster[j];
      }
    }

    // -----------------------------------------------------------------------------
    // ------ (3) Assign the particles to clusters
    // ------ (3) Equal-size K-means from https://stackoverflow.com/a/8810231
    // -----------------------------------------------------------------------------
    std::map<int, int> swap;
    for (auto& particle : particles_)
    {
      float min_dist = std::numeric_limits<float>::max();
      int which_cluster = -1;
      for (int cluster = 0; cluster < number_clusters_; cluster++)
      {
        float m_dist = particle.p_.distance(centroids[cluster]);
        if (m_dist < min_dist)
        {
          min_dist = m_dist;
          which_cluster = cluster;
        }
      }

      if (which_cluster != particle.which_cluster_)
      {
        auto it = swap.find(particle.which_cluster_);
        if (num_per_cluster[particle.which_cluster_] >
            static_cast<float>(params_.number_particles_) / static_cast<float>(number_clusters_))
        {
          num_per_cluster[which_cluster]++;
          num_per_cluster[particle.which_cluster_]--;
          particle.which_cluster_ = which_cluster;
          swaps++;
        }
        else if (it != swap.end() && particles_[it->second].which_cluster_ == which_cluster &&
                 it->second != particle.id_)
        {
          particles_[it->second].which_cluster_ = particle.which_cluster_;
          particle.which_cluster_ = which_cluster;
          swap.erase(particle.which_cluster_);
          swaps++;
        }
        else
        {
          swap[which_cluster] = particle.id_;
        }
      }
    }

    if (swaps == 0)
    {
      break;
    }
    else
    {
      swaps = 0;
    }
  }

  // -----------------------------------------------------------------------------
  // ------ (4) Compute weighted Gaussian approximation of each cluster
  // -----------------------------------------------------------------------------
  // ---- Weighted mean
  // - For the orientations - Mean of circular quantities
  // (https://en.wikipedia.org/wiki/Mean_of_circular_quantities)
  std::vector<Pose> means(number_clusters_, Pose(0., 0., 0., 0., 0., 0.));
  std::vector<Pose> means_sin(number_clusters_, Pose(0., 0., 0., 0., 0., 0.));
  std::vector<Pose> means_cos(number_clusters_, Pose(0., 0., 0., 0., 0., 0.));
  std::vector<float> sums(number_clusters_, 0.);
  for (const auto& particle : particles_)
  {
    means[particle.which_cluster_].x_ += (particle.w_ * particle.p_.x_);
    means[particle.which_cluster_].y_ += (particle.w_ * particle.p_.y_);
    means[particle.which_cluster_].z_ += (particle.w_ * particle.p_.z_);
    means_sin[particle.which_cluster_].R_ += (particle.w_ * std::sin(particle.p_.R_));
    means_cos[particle.which_cluster_].R_ += (particle.w_ * std::cos(particle.p_.R_));
    means_sin[particle.which_cluster_].P_ += (particle.w_ * std::sin(particle.p_.P_));
    means_cos[particle.which_cluster_].P_ += (particle.w_ * std::cos(particle.p_.P_));
    means_sin[particle.which_cluster_].Y_ += (particle.w_ * std::sin(particle.p_.Y_));
    means_cos[particle.which_cluster_].Y_ += (particle.w_ * std::cos(particle.p_.Y_));

    sums[particle.which_cluster_] += particle.w_;
  }
  for (int cluster = 0; cluster < number_clusters_; cluster++)
  {
    if (sums[cluster] > 0.)
    {
      means[cluster].x_ /= sums[cluster];
      means[cluster].y_ /= sums[cluster];
      means[cluster].z_ /= sums[cluster];
      means[cluster].R_ = std::atan2(means_sin[cluster].R_ / sums[cluster], means_cos[cluster].R_ / sums[cluster]);
      means[cluster].P_ = std::atan2(means_sin[cluster].P_ / sums[cluster], means_cos[cluster].P_ / sums[cluster]);
      means[cluster].Y_ = std::atan2(means_sin[cluster].Y_ / sums[cluster], means_cos[cluster].Y_ / sums[cluster]);
    }
  }

  // ---- Weighted standard deviation
  std::vector<Pose> stds(number_clusters_, Pose(0., 0., 0., 0., 0., 0.));
  std::vector<float> non_zero(number_clusters_, 0.);
  for (const auto& particle : particles_)
  {
    float w_x = particle.w_ * ((particle.p_.x_ - means[particle.which_cluster_].x_) *
                               (particle.p_.x_ - means[particle.which_cluster_].x_));
    float w_y = particle.w_ * ((particle.p_.y_ - means[particle.which_cluster_].y_) *
                               (particle.p_.y_ - means[particle.which_cluster_].y_));
    float w_z = particle.w_ * ((particle.p_.x_ - means[particle.which_cluster_].x_) *
                               (particle.p_.x_ - means[particle.which_cluster_].z_));
    float w_roll = particle.w_ * ((particle.p_.R_ - means[particle.which_cluster_].R_) *
                                  (particle.p_.R_ - means[particle.which_cluster_].R_));
    float w_pitch = particle.w_ * ((particle.p_.P_ - means[particle.which_cluster_].P_) *
                                   (particle.p_.P_ - means[particle.which_cluster_].P_));
    float w_yaw = particle.w_ * ((particle.p_.Y_ - means[particle.which_cluster_].Y_) *
                                 (particle.p_.Y_ - means[particle.which_cluster_].Y_));

    stds[particle.which_cluster_].x_ += w_x;
    stds[particle.which_cluster_].y_ += w_y;
    stds[particle.which_cluster_].z_ += w_z;
    stds[particle.which_cluster_].R_ += w_roll;
    stds[particle.which_cluster_].P_ += w_pitch;
    stds[particle.which_cluster_].Y_ += w_yaw;

    non_zero[particle.which_cluster_] = (particle.w_ > 0.) ?
                                            (non_zero[particle.which_cluster_] + static_cast<float>(1.)) :
                                            non_zero[particle.which_cluster_];
  }
  for (int cluster = 0; cluster < number_clusters_; cluster++)
  {
    if (sums[cluster] > 0. && non_zero[cluster] > 1.)
    {
      stds[cluster].x_ = std::sqrt(stds[cluster].x_ / ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));
      stds[cluster].y_ = std::sqrt(stds[cluster].y_ / ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));
      stds[cluster].z_ = std::sqrt(stds[cluster].z_ / ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));
      stds[cluster].R_ = std::sqrt(stds[cluster].R_ / ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));
      stds[cluster].P_ = std::sqrt(stds[cluster].P_ / ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));
      stds[cluster].Y_ = std::sqrt(stds[cluster].Y_ / ((non_zero[cluster] - 1) / non_zero[cluster] * sums[cluster]));

      Gaussian<Pose, Pose> m_gaussian(means[cluster], stds[cluster]);
      gauss_map[cluster] = m_gaussian;
    }
  }
}

void PF::scanMatch(const std::vector<ImageFeature>& features, OccupancyMap* grid_map,
                   std::map<int, Gaussian<Pose, Pose>>& gauss_map, std::vector<float>& ws)
{
  float normalizer_icp = static_cast<float>(1.) / (sigma_feature_matching_ * std::sqrt(M_2PI));

  std::map<int, Tf> tfs;
  // -------------------------------------------------------------------------------
  // ------- 3D scan matching using low level features
  // -------------------------------------------------------------------------------
  // - Set target point clouds (common to all clusters)
  icp_->setInputTarget(grid_map);
  icp_->setInputSource(features);
  // - Perform scan matching for each cluster
  bool valid_it = false;
  std::map<int, float> cluster_ws;
  for (auto& it : gauss_map)
  {
    // Convert cluster pose to [R|t]
    std::array<float, 3> trans = { it.second.mean_.x_, it.second.mean_.y_, it.second.mean_.z_ };
    std::array<float, 9> Rot{};
    it.second.mean_.toRotMatrix(Rot);

    // --------------- Perform scan matching ---------------------
    std::vector<ImageFeature> aligned;
    float rms_error;
    Tf m_tf;
    Tf final_tf;
    Tf original_tf(Rot, trans);
    // - First guess: each particle drawn by odometry motion model
    // - Only use scan match if it does no fail
    if (icp_->align(original_tf, rms_error, aligned))
    {
      // ---------------------------------------------------------------------------
      // ------------ Get homogeneous transformation result
      // ---------------------------------------------------------------------------
      icp_->getTransform(m_tf);

      final_tf = original_tf.inverse() * m_tf;

      // ---------------------------------------------------------------------------
      // ----------- Compute scan match weights
      // ---------------------------------------------------------------------------
      // - Get the correspondences errors both spatial and for the descriptors
      std::vector<float> serror;
      icp_->getErrors(serror);

      // - Prevent single correspondence - standard deviation = 0
      if (serror.size() <= 1)
      {
        final_tf.R_array_ = std::array<float, 9>{ 1., 0., 0., 0., 1., 0., 0., 0., 1. };
        final_tf.t_array_ = std::array<float, 3>{ 0., 0., 0. };
        ws[it.first] = 0.;
        continue;
      }

      // - Compute weight
      float w = 0.;
      for (float i : serror)
      {
        w += static_cast<float>(normalizer_icp * exp(-1. / sigma_feature_matching_ * i));
      }

      cluster_ws[it.first] = w;
      valid_it = true;
    }
    else
    {
      final_tf = Tf::unitary();
      ws[it.first] = 0.;
    }

    // Get delta transform
    tfs[it.first] = final_tf;
  }

  // -------------------------------------------------------------------------------
  // ------- Apply ICP result to the clusters and update particles weights
  // -------------------------------------------------------------------------------
  if (valid_it)
  {
    for (auto& particle : particles_)
    {
      // - ICP result application
      Tf m_tf = tfs[particle.which_cluster_];

      particle.tf_ = particle.tf_ * m_tf;
      particle.p_ = Pose(particle.tf_.R_array_, particle.tf_.t_array_);
      particle.p_.normalize();

      // Update particle weight
      ws[particle.id_] = cluster_ws[particle.which_cluster_];
    }
  }
}

void PF::normalizeWeights()
{
  if (w_sum_ > 0.)
  {
    for (auto& particle : particles_)
      particle.w_ /= w_sum_;
  }
  else
  {
    for (auto& particle : particles_)
      particle.w_ = static_cast<float>(1.) / static_cast<float>(particles_.size());
  }
}

void PF::resample()
{
  float cweight = 0.;
  uint32_t n = particles_.size();

  // - Compute the cumulative weights
  for (const auto& p : particles_)
    cweight += p.w_;
  // - Compute the interval
  float interval = cweight / n;
  // - Compute the initial target weight
  auto target = static_cast<float>(interval * ::drand48());

  // - Compute the resampled indexes
  cweight = 0.;
  std::vector<uint32_t> indexes(n);
  n = 0.;
  uint32_t i = 0;

  for (const auto& p : particles_)
  {
    cweight += p.w_;
    while (cweight > target)
    {
      indexes[n++] = i;
      target += interval;
    }

    i++;
  }

  // - Update particle set
  for (size_t j = 0; j < indexes.size(); j++)
  {
    particles_[j].pp_ = particles_[indexes[j]].pp_;
    particles_[j].ptf_ = particles_[indexes[j]].ptf_;
    particles_[j].p_ = particles_[indexes[j]].p_;
    particles_[j].tf_ = particles_[indexes[j]].tf_;
    particles_[j].w_ = particles_[indexes[j]].w_;
    particles_[j].which_cluster_ = particles_[indexes[j]].which_cluster_;
  }
}

}  // namespace vineslam
