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
  srr     = config["pf"]["srr"].as<float>();
  str     = config["pf"]["str"].as<float>();
  stt     = config["pf"]["stt"].as<float>();
  srt     = config["pf"]["srt"].as<float>();
  sigma_z = config["pf"]["sigma_z"].as<float>();
  sigma_roll =
      config["pf"]["sigma_roll"].as<float>() * static_cast<float>(PI / 180.);
  sigma_pitch =
      config["pf"]["sigma_pitch"].as<float>() * static_cast<float>(PI / 180.);
  n_particles = config["pf"]["n_particles"].as<float>();

  // Set particles to zero
  particles.resize(n_particles);
  for (size_t i = 0; i < particles.size(); i++) {
    particles[i].id = i;
    particles[i].p  = pose(0., 0., 0., 0., 0., 0.);
    particles[i].w  = static_cast<float>(1. / n_particles);
  }

  // Initialize particles
  p_odom = pose(0., 0., 0.);
  motionModel(initial_pose);

  // Initialize the previous pose
  p_odom = initial_pose;
}

void PF::motionModel(const pose& odom)
{
  // 3D particle filter
  for (auto& particle : particles) {
    drawFromMotion(odom, particle.p);

    // Set unobservable components - draw from zero-mean gaussian distribution
    particle.p.z += 0.;
    particle.p.roll += 0.;
    particle.p.pitch += 0.;
    particle.p.pitch = cam_pitch;
  }
}

void PF::drawFromMotion(const pose& odom, pose& p) const
{
  float sxy = static_cast<float>(.3) * srr;

  // Compute delta pose between last and current control
  pose dt_pose = odom - p_odom;
  dt_pose.yaw  = std::atan2(std::sin(dt_pose.yaw), std::cos(dt_pose.yaw));
  float s1 = std::sin(p_odom.yaw), c1 = std::cos(p_odom.yaw);
  // ------------------------------------------------------
  pose delta;
  delta.x   = c1 * dt_pose.x + s1 * dt_pose.y;
  delta.y   = -s1 * dt_pose.x + c1 * dt_pose.y;
  delta.yaw = dt_pose.yaw;
  // ------------------------------------------------------
  // Sample zero mean gaussian to compute noisy pose
  pose noisypoint(delta);
  noisypoint.x +=
      sampleGaussian(srr * std::fabs(delta.x) + str * std::fabs(delta.yaw) +
                     sxy * std::fabs(delta.y));
  noisypoint.y +=
      sampleGaussian(srr * std::fabs(delta.y) + str * std::fabs(delta.yaw) +
                     sxy * std::fabs(delta.x));
  noisypoint.yaw +=
      sampleGaussian(stt * std::fabs(delta.yaw) +
                     srt * std::sqrt(delta.x * delta.x + delta.y * delta.y));
  // -------------------------------------------------------------------------------
  noisypoint.normalize();
  // -------------------------------------------------------------------------------

  // Apply motion model
  // ------------------------------------------------------
  float s2 = std::sin(p.yaw), c2 = std::cos(p.yaw);
  p.x += c2 * noisypoint.x - s2 * noisypoint.y;
  p.y += s2 * noisypoint.x + c2 * noisypoint.y;
  p.yaw += noisypoint.yaw;
  // ------------------------------------------------------
}

void PF::update(const std::vector<SemanticFeature>& landmarks,
                const std::vector<Corner>&          corners,
                const Plane&                        ground_plane,
                OccupancyMap                        grid_map)
{
  // Loop over all particles
  for (auto& particle : particles) {
    // Convert particle orientation to rotation matrix
    std::array<float, 9> Rot{};
    particle.p.toRotMatrix(Rot);

    // ------------------------------------------------------
    // --- 2D semantic feature map fitting
    // ------------------------------------------------------
    float              dmean = 0.;
    std::vector<float> dvec;
    for (const auto& landmark : landmarks) {
      // Convert landmark to the particle's referential frame
      point X;
      X.x = landmark.pos.x * Rot[0] + landmark.pos.y * Rot[1] +
            landmark.pos.z * Rot[2] + particle.p.x;
      X.y = landmark.pos.x * Rot[3] + landmark.pos.y * Rot[4] +
            landmark.pos.z * Rot[5] + particle.p.y;
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
      else {
        dmean += best_correspondence;
        dvec.push_back(best_correspondence);
      }
    }

    // ------------------------------------------------------
    // --- Particle weight update
    // ------------------------------------------------------
    if (dvec.size() <= 1) {
      particle.w = 0.;
      continue;
    }
    // Compute mean and standard deviation of the correspondences
    dmean /= static_cast<float>(dvec.size());
    float w      = 0.;
    float dstdev = 0.;
    for (const auto& dist : dvec) dstdev += std::pow(dist - dmean, 2);
    dstdev /= static_cast<float>(dvec.size());
    // Update particle weight
    for (const auto& dist : dvec)
      w += static_cast<float>((1. / (std::sqrt(2. * PI) * dstdev)) *
                              std::exp(-dist / (2. * PI * dstdev * dstdev)));
    particle.w = w;
    w_sum += w;
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
    std::cout << j << " - > " << particles[j].w << std::endl;
  }
}

} // namespace vineslam
