#include "../../include/vineslam/localization/pf.hpp"

namespace vineslam
{
PF::PF(const Parameters& params, const Pose& initial_pose) : params_(params)
{
  // - General parameters
  use_semantic_features_ = params.use_semantic_features_;
  use_lidar_features_ = params.use_lidar_features_;
  use_gps_ = params.use_gps_;
  use_gps_altitude_ = params.use_gps_altitude_;
  use_imu_ = params.use_imu_;
  particles_size_ = params.number_particles_;

  // Initialize thread pool
  thread_pool_ = new lama::ThreadPool;
  thread_pool_->init(NUM_THREADS);

  // Initialize profiler
  t_ = new Timer("Particle Filter");

  // Initialize normal distributions
  particles_.resize(particles_size_);

  // Initialize all particles
  for (size_t i = 0; i < particles_size_; i++)
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
  sigma_landmark_matching_ = 0.1;
  sigma_feature_matching_ = 0.1;
  sigma_corner_matching_ = 0.1;
  sigma_planar_matching_ = 0.1;
  sigma_plane_matching_vector_ = 0.02;
  sigma_plane_matching_centroid_ = 0.10;
  sigma_gps_ = 0.05;
  sigma_imu_ = 5 * DEGREE_TO_RAD;
}

// Samples a zero mean Gaussian
// See https://www.taygeta.com/random/gaussian.html
float PF::sampleGaussian(const float& sigma, const unsigned long int& S)
{
  if (S != 0)
    srand48(S);
  if (sigma == 0)
    return 0.;

  float x1, x2, w;
  float r;

  do
  {
    do
    {
      r = drand48();
    } while (r == 0.0);
    x1 = 2.0 * r - 1.0;
    do
    {
      r = drand48();
    } while (r == 0.0);
    x2 = 2.0 * drand48() - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w > 1.0 || w == 0.0);

  return (sigma * x2 * sqrt(-2.0 * log(w) / w));
}

void PF::motionModel(const Pose& odom_inc)
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
      i = (d_trans /* + d_rot*/) * sampleGaussian(1.0);

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
                const Pose& gps_pose, const Pose& imu_pose, OccupancyMap* grid_map)
{
  std::vector<float> semantic_weights(particles_size_, 0.);
  std::vector<float> corner_weights(particles_size_, 0.);
  std::vector<float> planar_weights(particles_size_, 0.);
  std::vector<float> planes_weights(particles_size_, 0.);
  std::vector<float> ground_weights(particles_size_, 0.);
  std::vector<float> surf_weights(particles_size_, 0.);
  std::vector<float> gps_weights(particles_size_, 0.);
  std::vector<float> imu_weights(particles_size_, 0.);

  // if (use_semantic_features_)
  if (0)
  {
    t_->tick("pf::semantic()");
    highLevel(landmarks, grid_map, semantic_weights);
    t_->tock();
  }

  if (use_lidar_features_)
  {
    t_->tick("pf::corners()");
    mediumLevelCorners(corners, grid_map, corner_weights);
    t_->tock();

    t_->tick("pf::planars()");
    mediumLevelPlanars(planars, grid_map, planar_weights);
    t_->tock();

    t_->tick("pf::planes()");
    mediumLevelPlanes({ ground_plane }, grid_map, ground_weights);
    mediumLevelPlanes(planes, grid_map, planes_weights);
    t_->tock();
  }

  if (use_gps_)
  {
    gps(gps_pose, gps_weights);
  }

  if (use_imu_)
  {
    imu(imu_pose, imu_weights);
  }

  // Multi-modal weights normalization
  float semantic_max = *std::max_element(semantic_weights.begin(), semantic_weights.end());
  float corners_max = *std::max_element(corner_weights.begin(), corner_weights.end());
  float planars_max = *std::max_element(planar_weights.begin(), planar_weights.end());
  float planes_max = *std::max_element(planes_weights.begin(), planes_weights.end());
  float ground_max = *std::max_element(ground_weights.begin(), ground_weights.end());
  float surf_max = *std::max_element(surf_weights.begin(), surf_weights.end());
  float gps_max = *std::max_element(gps_weights.begin(), gps_weights.end());
  float imu_max = *std::max_element(imu_weights.begin(), imu_weights.end());
  for (auto& particle : particles_)
  {
    float m_lw = (semantic_max > 0.) ? semantic_weights[particle.id_] : static_cast<float>(1.);
    float m_cw = (corners_max > 0.) ? corner_weights[particle.id_] : static_cast<float>(1.);
    float m_rw = (planars_max > 0.) ? planar_weights[particle.id_] : static_cast<float>(1.);
    float m_pw = (planes_max > 0.) ? planes_weights[particle.id_] : static_cast<float>(1.);
    float m_gw = (ground_max > 0.) ? ground_weights[particle.id_] : static_cast<float>(1.);
    float m_sw = (surf_max > 0.) ? surf_weights[particle.id_] : static_cast<float>(1.);
    float m_gpsw = (gps_max > 0.) ? gps_weights[particle.id_] : static_cast<float>(1.);
    float m_iw = (imu_max > 0.) ? imu_weights[particle.id_] : static_cast<float>(1.);

    particle.w_ = m_lw * m_cw * m_rw * m_pw * m_gw * m_sw * m_gpsw * m_iw;

    w_sum_ += particle.w_;
  }

  t_->getLog();
  t_->clearLog();
}

void PF::updateModel(const float& z_k, const float& z_k_asterisc, const float& z_dist, const float& sigma_hit,
                     const float& sigma_short, float& w)
{
  // Define weights of each of the model components considering that
  // z_hit + z_short + z_max + z_rand = 1
  float z_hit = 0.4;
  float z_short = 0.3;
  float z_max = 0.2;
  float z_rand = 0.1;

  // Maximum distance measured by the LiDAR
  float d_max = 200;
  // Distance to the features

  // Compute the 'correct range with local measurement noise' component (hit)
  float normalizer_hit = static_cast<float>(1.) / (sigma_hit * std::sqrt(M_2PI));
  float squared_dist_hit = std::pow(z_dist, 2);
  float p_hit = (normalizer_hit * static_cast<float>(std::exp(-1. / sigma_hit * squared_dist_hit)));

  // Compute the 'unexpected objects component' (short)
  float normalizer_short = 1 / (1 - std::exp(-sigma_short * z_k_asterisc));
  float p_short = normalizer_short * sigma_short * std::exp(-sigma_short * z_k);

  // Compute the 'failures' component (max)
  float p_max = (z_k >= d_max) ? 1. : 0.;

  // Compute the 'random measurements' component (rand)
  float p_rand = (z_k >= 0 && z_k < d_max) ? (1. / d_max) : 0.;

  // Compute the final weight
  w = (z_hit * p_hit) + (z_short * p_short) + (z_max * p_max) + (z_rand * p_rand);
}

void PF::gps(const Pose& gps_pose, std::vector<float>& ws)
{
  float normalizer_gps = static_cast<float>(1.) / (sigma_gps_ * std::sqrt(M_2PI));

  for (const auto& particle : particles_)
  {
    // - GPS [x, y] weight
    float w_gps, dist;
    if (use_gps_altitude_)
    {
      dist = particle.p_.distance(gps_pose);
    }
    else
    {
      dist = particle.p_.distanceXY(gps_pose);
    }
    w_gps = (normalizer_gps * static_cast<float>(std::exp(-1. / sigma_gps_ * dist)));

    ws[particle.id_] = w_gps;
  }
}

void PF::imu(const Pose& imu_pose, std::vector<float>& ws)
{
  float normalizer_imu = static_cast<float>(1.) / (sigma_imu_ * std::sqrt(M_2PI));

  for (const auto& particle : particles_)
  {
    // - IMU [roll, pitch] weight
    float w_imu;
    float delta_R = std::fabs(Const::normalizeAngle(particle.p_.R_ - imu_pose.R_));
    float delta_P = std::fabs(Const::normalizeAngle(particle.p_.P_ - imu_pose.P_));

    w_imu = (normalizer_imu * static_cast<float>(std::exp(-1. / sigma_imu_ * delta_R))) *
            (normalizer_imu * static_cast<float>(std::exp(-1. / sigma_imu_ * delta_P)));

    ws[particle.id_] = w_imu;
  }
}

void PF::highLevel(const std::vector<SemanticFeature>& landmarks, OccupancyMap* grid_map, std::vector<float>& ws)
{
  float normalizer_landmark = static_cast<float>(1.) / (sigma_landmark_matching_ * std::sqrt(M_2PI));

  // Loop over all particles
  for (uint32_t i = 0; i < particles_size_; ++i)
  {
#if NUM_THREADS > 1
    thread_pool_->enqueue([this, landmarks, grid_map, normalizer_landmark, &ws, i]() {
#endif
      // Convert particle orientation to rotation matrix
      Pose l_pose = particles_[i].p_;
      l_pose.R_ = 0.;
      l_pose.P_ = 0.;
      l_pose.z_ = 0.;
      std::array<float, 9> Rot{};
      l_pose.toRotMatrix(Rot);

      // ------------------------------------------------------
      // --- 2D semantic feature map fitting
      // ------------------------------------------------------
      float w_landmarks = 0.;
      for (const auto& landmark : landmarks)
      {
        // Convert landmark to the maps's referential frame
        Point X;
        X.x_ = landmark.pos_.x_ * Rot[0] + landmark.pos_.y_ * Rot[1] + landmark.pos_.z_ * Rot[2] + l_pose.x_;
        X.y_ = landmark.pos_.x_ * Rot[3] + landmark.pos_.y_ * Rot[4] + landmark.pos_.z_ * Rot[5] + l_pose.y_;
        X.z_ = 0.;

        // Search for a correspondence in the current cell first
        float best_correspondence = std::numeric_limits<float>::max();
        bool found = false;

        // Check cell data
        Cell* c = &(*grid_map)(X.x_, X.y_, 0);
        if (c->data == nullptr)
        {
          continue;
        }
        std::map<int, SemanticFeature>* l_landmarks = c->data->landmarks_;
        if (l_landmarks == nullptr)
        {
          continue;
        }

        for (const auto& l_landmark : *l_landmarks)
        {
          float dist_min = X.distanceXY(l_landmark.second.pos_);

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

          for (const auto& l_cell : adjacents)
          {
            // Check cell data
            if (l_cell.data == nullptr)
            {
              continue;
            }
            std::map<int, SemanticFeature>* ll_landmarks = l_cell.data->landmarks_;
            if (ll_landmarks == nullptr)
            {
              continue;
            }

            for (const auto& l_landmark : *ll_landmarks)
            {
              float dist_min = X.distanceXY(l_landmark.second.pos_);
              if (dist_min < best_correspondence)
              {
                best_correspondence = dist_min;
                found = true;
              }
            }
          }
        }

        // Save distance if a correspondence was found
        if (found)
          w_landmarks += (normalizer_landmark *
                          static_cast<float>(std::exp(-1. / sigma_landmark_matching_ * best_correspondence)));
      }

      ws[particles_[i].id_] = w_landmarks;

#if NUM_THREADS > 1
    });
#endif
  }

#if NUM_THREADS > 1
  thread_pool_->wait();
#endif
}

void PF::mediumLevelCorners(const std::vector<Corner>& corners, OccupancyMap* grid_map, std::vector<float>& ws)
{
  float normalizer_corner = static_cast<float>(1.) / (sigma_corner_matching_ * std::sqrt(M_2PI));

  // Loop over all particles
  for (uint32_t i = 0; i < particles_size_; ++i)
  {
#if NUM_THREADS > 1
    thread_pool_->enqueue([this, corners, grid_map, normalizer_corner, &ws, i]() {
#endif
      // ------------------------------------------------------
      // --- 3D corner map fitting
      // ------------------------------------------------------
      float w_corners = 0;
      for (const auto& corner : corners)
      {
        // Convert feature to the map's referential frame
        Point X = corner.pos_ * particles_[i].tf_;

        // Check cell data
        Cell* c = &(*grid_map)(X.x_, X.y_, X.z_);
        if (c->data == nullptr)
        {
          continue;
        }
        std::vector<Corner>* l_corners = c->data->corner_features_;
        if (l_corners == nullptr)
        {
          continue;
        }

        // Search for a correspondence in the current cell first
        Point best_correspondence_point;
        float best_correspondence = 0.5;
        bool found = false;
        for (const auto& l_corner : *l_corners)
        {
          float dist_sq = ((X.x_ - l_corner.pos_.x_) * (X.x_ - l_corner.pos_.x_) +
                           (X.y_ - l_corner.pos_.y_) * (X.y_ - l_corner.pos_.y_) +
                           (X.z_ - l_corner.pos_.z_) * (X.z_ - l_corner.pos_.z_));

          if (dist_sq < best_correspondence)
          {
            best_correspondence_point = l_corner.pos_;
            best_correspondence = dist_sq;
            found = true;
          }
        }

        // Save distance if a correspondence was found
        if (found)
        {
          w_corners +=
              (normalizer_corner * static_cast<float>(std::exp(-1. / sigma_corner_matching_ * best_correspondence)));
          // float l_w;
          // updateModel(X.norm3D(), best_correspondence_point.norm3D(), best_correspondence, sigma_corner_matching_,
          // 0.1, l_w);
          // w_corners += l_w;
        }
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
  for (uint32_t i = 0; i < particles_size_; ++i)
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

        // Check cell data
        Cell* c = &(*grid_map)(X.x_, X.y_, X.z_);
        if (c->data == nullptr)
        {
          continue;
        }
        std::vector<Planar>* l_planars = c->data->planar_features_;
        if (l_planars == nullptr)
        {
          continue;
        }

        // Search for a correspondence in the current cell first
        Point best_correspondence_point;
        float best_correspondence = 0.5;
        bool found = false;
        for (const auto& l_planar : *l_planars)
        {
          float dist_sq = ((X.x_ - l_planar.pos_.x_) * (X.x_ - l_planar.pos_.x_) +
                           (X.y_ - l_planar.pos_.y_) * (X.y_ - l_planar.pos_.y_) +
                           (X.z_ - l_planar.pos_.z_) * (X.z_ - l_planar.pos_.z_));

          if (dist_sq < best_correspondence)
          {
            best_correspondence_point = l_planar.pos_;
            best_correspondence = dist_sq;
            found = true;
          }
        }

        // Save distance if a correspondence was found
        if (found)
        {
          w_planars +=
              (normalizer_planar * static_cast<float>(std::exp((-1. / sigma_planar_matching_) * best_correspondence)));

          // float l_w;
          // updateModel(X.norm3D(), best_correspondence_point.norm3D(), best_correspondence, sigma_planar_matching_,
          // 0.1, l_w);
          // w_planars += l_w;
        }
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
  for (uint32_t i = 0; i < particles_size_; ++i)
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
        l_plane.centroid_ = l_plane.centroid_ * particles_[i].tf_;  // Convert the centroid
        Ransac::estimateNormal(l_plane.points_, l_plane.a_, l_plane.b_, l_plane.c_,
                               l_plane.d_);  // Convert plane normal

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
          ConvexHull::polygonIntersection(gg_plane, lg_plane, isct.extremas_);

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

        if (found)
        {
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
      particle.w_ = static_cast<float>(1.) / static_cast<float>(particles_size_);
  }
}

void PF::resample()
{
  float cweight = 0.;
  uint32_t n = particles_size_;

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
