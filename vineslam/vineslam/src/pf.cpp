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
  n_particles             = config["pf"]["n_particles"].as<float>();

  // Initialize normal distributions
  particles.resize(n_particles);

  // Initialize all particles
  for (size_t i = 0; i < particles.size(); i++) {
    // Calculate the initial pose for each particle considering
    // - the input initial pose
    // - a sample distribution to spread the particles
    pose m_pose(sampleGaussian(sigma_xy),
                sampleGaussian(sigma_xy),
                sampleGaussian(sigma_z),
                sampleGaussian(sigma_roll),
                cam_pitch + sampleGaussian(sigma_pitch),
                sampleGaussian(sigma_yaw));
    // Compute initial weight of each particle
    float weight = 1.;
    // Insert the particle into the particles array
    particles[i] = Particle(i, m_pose, weight);
  }

  // Initialize the previous pose
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
                   ? static_cast<float>(PI)
                   : 0.; // PREVENT ERROR WHEN ROBOT IS MOVING BACKWARDS :-)
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
    dt_pose.x = s_trans * std::cos(normalizeAngle(particle.p.yaw + s_rot_a)) +
                sampleGaussian(0.001);
    dt_pose.y = s_trans * std::sin(normalizeAngle(particle.p.yaw + s_rot_a)) +
                sampleGaussian(0.001);
    dt_pose.z     = sampleGaussian(sigma_z);
    dt_pose.roll  = sampleGaussian(sigma_roll);
    dt_pose.pitch = sampleGaussian(sigma_pitch);
    dt_pose.yaw   = s_rot_a + s_rot_b + sampleGaussian(0.0005);

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
  // Loop over all particles
  for (auto& particle : particles) {
    // Convert particle orientation to rotation matrix
    pose m_pose = particle.p;
    // Use only 2D pose estimation - set roll, pitch, z to 0
    // TODO (André Aguiar): Here we should use roll and pitch, but we're not getting
    // good results with them ...
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
      ;
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
        grid_map.getAdjacent(X.x, X.y, 2, adjacents);
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
    // - Semantic landmark matching weight
    if (dlandmarkvec.size() <= 1) {
      particle.w = 0.;
    } else {
      float w = 0.;
      for (const auto& dist : dlandmarkvec)
        w += static_cast<float>(std::exp(-1. / sigma_landmark_matching * dist));
      particle.w *= w;
    }
    // - Corner feature mathing weight
    if (dcornervec.size() <= 1) {
      particle.w = 0.;
    } else {
      float w = 0.;
      for (const auto& dist : dcornervec)
        w += static_cast<float>(std::exp(-1. / sigma_corner_matching * dist));
      particle.w *= w;
    }
    // - Add random noise to the particle weigth
    particle.w += sampleGaussian(0.01);

    w_sum += particle.w;
  }
}

void PF::normalizeWeights()
{
  if (w_sum > 0.) {
    for (auto& particle : particles) {
      particle.w /= w_sum;
    }
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

} // namespace vineslam
