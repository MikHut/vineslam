#include "pf.hpp"

#include <cmath>

namespace vineslam
{

PF::PF(const std::string& config_path, const pose& initial_pose)
    : config_path(config_path)
{
  // Read input parameters
  YAML::Node config = YAML::LoadFile(config_path);
  cam_pitch =
      config["camera_info"]["cam_pitch"].as<float>() * static_cast<float>(PI / 180.);
  srr      = config["pf"]["srr"].as<float>();
  str      = config["pf"]["str"].as<float>();
  stt      = config["pf"]["stt"].as<float>();
  srt      = config["pf"]["srt"].as<float>();
  sigma_xy = config["pf"]["sigma_xy"].as<float>();
  sigma_z  = config["pf"]["sigma_z"].as<float>();
  sigma_roll =
      config["pf"]["sigma_roll"].as<float>() * static_cast<float>(PI / 180.);
  sigma_pitch =
      config["pf"]["sigma_pitch"].as<float>() * static_cast<float>(PI / 180.);
  sigma_yaw = config["pf"]["sigma_yaw"].as<float>() * static_cast<float>(PI / 180.);
  sigma_landmark_matching = config["pf"]["sigma_landmark_matching"].as<float>();
  sigma_feature_matching  = config["pf"]["sigma_feature_matching"].as<float>();
  sigma_corner_matching   = config["pf"]["sigma_corner_matching"].as<float>();
  sigma_ground_z          = config["pf"]["sigma_ground_rp"].as<float>();
  sigma_ground_rp =
      config["pf"]["sigma_ground_z"].as<float>() * static_cast<float>(PI / 180.);
  n_particles = config["pf"]["n_particles"].as<float>();

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
    particles[i] = Particle(i, m_pose, pose(0., 0., 0., 0., 0., 0.), weight);
  }

  // Set last iteration vars
  p_odom              = initial_pose;
  last_ground_plane_z = 0.;
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
                   ? PI
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

    // Save last pose before inovation
    particle.last_p = particle.p;
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

void PF::update(const std::vector<SemanticFeature>& landmarks,
                const std::vector<Corner>&          corners,
                const Plane&                        ground_plane,
                OccupancyMap                        grid_map)
{
  // -------------------------------------------------------------------------------
  // --- 3D ground plane [roll, pitch, z] estimation - only done once
  // -------------------------------------------------------------------------------
  pose                   pose_from_ground;
  Gaussian<float, float> ground_plane_gauss{};
  if (!ground_plane.points.empty()) {
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

  // COMPUTE STATIC VARIABLES TO USE IN THE PARTICLES LOOP
  float normalizer_landmark =
      static_cast<float>(1.) / (sigma_landmark_matching * std::sqrt(2 * PI));
  float normalizer_corner =
      static_cast<float>(1.) / (sigma_feature_matching * std::sqrt(2 * PI));
  float delta_ground_z = pose_from_ground.z - last_ground_plane_z;
  float normalizer_ground_z =
      static_cast<float>(1.) / (sigma_ground_z * std::sqrt(2 * PI));
  float normalizer_ground_rp =
      static_cast<float>(1.) / (sigma_ground_rp * std::sqrt(2 * PI));
  // -----------------------------------------------------

  // Loop over all particles
  for (auto& particle : particles) {
    // Convert particle orientation to rotation matrix
    pose m_pose = particle.p;
    m_pose.z    = 0.;
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
//    Rot = {}; // clear rotation matrix to set for 3D estimation
//    particle.p.toRotMatrix(Rot);
//    std::vector<float> dcornervec;
//    for (const auto& corner : corners) {
//      // Convert landmark to the particle's referential frame
//      point X;
//      X.x = corner.pos.x * Rot[0] + corner.pos.y * Rot[1] + corner.pos.z * Rot[2] +
//            particle.p.x;
//      X.y = corner.pos.x * Rot[3] + corner.pos.y * Rot[4] + corner.pos.z * Rot[5] +
//            particle.p.y;
//      X.z = corner.pos.x * Rot[6] + corner.pos.y * Rot[7] + corner.pos.z * Rot[8] +
//            particle.p.z;
//
//      // Search for a correspondence in the current cell first
//      float best_correspondence = std::numeric_limits<float>::max();
//      bool  found               = false;
//      for (const auto& m_corner : grid_map(X.x, X.y).corner_features) {
//        float dist_min = X.distance(m_corner.pos);
//
//        if (dist_min < best_correspondence) {
//          best_correspondence = dist_min;
//          found               = true;
//        }
//      }
//
//      // Only search in the adjacent cells if we do not find in the source cell
//      if (!found) {
//        std::vector<Cell> adjacents;
//        grid_map.getAdjacent(X.x, X.y, 1, adjacents);
//        for (const auto& m_cell : adjacents) {
//          for (const auto& m_corner : m_cell.corner_features) {
//            float dist_min = X.distance(m_corner.pos);
//            if (dist_min < best_correspondence) {
//              best_correspondence = dist_min;
//              found               = true;
//            }
//          }
//        }
//      }
//
//      // Save distance if a correspondence was found
//      if (!found)
//        continue;
//      else
//        dcornervec.push_back(best_correspondence);
//    }

    // ------------------------------------------------------
    // --- Particle weight update
    // ------------------------------------------------------
    // - Semantic landmark matching weight
    float w_landmarks = 0.;
    if (dlandmarkvec.size() <= 1) {
      w_landmarks = 0.;
    } else {
      for (const auto& dist : dlandmarkvec)
        w_landmarks +=
            (normalizer_landmark *
             static_cast<float>(std::exp(-1. / sigma_landmark_matching * dist)));
    }
    // - Corner feature matching weight
//    float w_corners = 0.;
//    if (dcornervec.size() <= 1) {
//      w_corners = 0.;
//    } else {
//
//      for (const auto& dist : dcornervec) {
//        w_corners +=
//            (normalizer_corner *
//             static_cast<float>(std::exp(-1. / sigma_corner_matching * dist)));
//      }
//    }
    // - Ground plane [roll, pitch, z] weight
    float w_ground = 0.;
    if (ground_plane_gauss.mean != 0. && ground_plane_gauss.stdev != 0.) {
      float delta_particle_z = particle.p.z - particle.last_p.z;
      w_ground =
          (normalizer_ground_z *
           static_cast<float>(
               std::exp(-1. / sigma_ground_z *
                        std::fabs(delta_ground_z - delta_particle_z)))) *
          (normalizer_ground_rp *
           static_cast<float>(
               std::exp(-1. / sigma_ground_rp *
                        std::fabs(particle.p.roll - pose_from_ground.roll)))) *
          (normalizer_ground_rp *
           static_cast<float>(
               std::exp(-1. / sigma_ground_rp *
                        std::fabs(particle.p.pitch - pose_from_ground.pitch))));
    }

    // Weights update heuristic
    float w = 0.;
//    if (w_landmarks > 0. && w_corners > 0.) {
//      w = w_landmarks * (w_corners * dlandmarkvec.size() / dcornervec.size());
//    } else if (w_landmarks == 0. && w_corners > 0. && dlandmarkvec.empty()) {
//      w = w_corners;
//    } else if (w_landmarks > 0. && w_corners == 0. && dcornervec.empty()) {
//      w = w_landmarks;
//    } else {
//      w = 0.;
//    }
    if (w_landmarks > 0. && ground_plane.points.size() > 1)
      w = (w_ground * w_landmarks);
    else
      w = 0.;

    float random_noise = sampleGaussian(0.001);
    particle.w         = particle.w * w; // + random_noise;

    w_sum += particle.w;
  }

  last_ground_plane_z = pose_from_ground.z;
}

void PF::normalizeWeights()
{
  if (w_sum > 0.) {
    for (auto& particle : particles) {
      particle.w /= w_sum;
      //      std::cout << particle.id << " - PITCH = " << particle.p.pitch << " -
      //      WEIGHT = " << particle.w << std::endl;
    }
  } else {
    for (auto& particle : particles)
      particle.w = static_cast<float>(1.) / static_cast<float>(particles.size());
  }
  //  std::cout << w_sum << std::endl;
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
    //    std::cout << j << " - Pitch: " << particles[j].p.pitch
    //              << " - w: " << particles[j].w;
    particles[j].p = particles[indexes[j]].p;
    particles[j].w = particles[indexes[j]].w;
    //    std::cout << " ---> " << particles[indexes[j]].w
    //              << " - Pitch: " << particles[indexes[j]].p.pitch << std::endl;
  }
}

} // namespace vineslam
