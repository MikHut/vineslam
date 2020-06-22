#include "pf.hpp"

namespace vineslam
{

PF::PF(const std::string& config_path, const pose& initial_pose)
    : config_path(config_path)
{
  // Read input parameters
  YAML::Node config  = YAML::LoadFile(config_path);
  srr                = config["pf"]["srr"].as<float>();
  str                = config["pf"]["str"].as<float>();
  stt                = config["pf"]["stt"].as<float>();
  srt                = config["pf"]["srt"].as<float>();
  sigma_z            = config["pf"]["sigma_z"].as<float>();
  sigma_roll         = config["pf"]["sigma_roll"].as<float>() * PI / 180.;
  sigma_pitch        = config["pf"]["sigma_pitch"].as<float>() * PI / 180.;
  n_particles2D      = config["pf"]["n_particles2D"].as<float>();
  n_particles3D      = config["pf"]["n_particles3D"].as<float>();
  auto icp_max_iters = config["ICP"]["max_iters"].as<float>();
  auto icp_tolerance = config["ICP"]["tolerance"].as<float>();
  auto dthreshold    = config["ICP"]["distance_threshold"].as<float>();

  // Initialize and set ICP parameters
  // ------------ Feature-based 3D ICP
  icp = new ICP(config_path);
  icp->setMaxIterations(static_cast<int>(icp_max_iters));
  icp->setTolerance(icp_tolerance);
  icp->setThreshold(dthreshold);
  // ---------------------------------

  // Set rms error arrays size
  rms_error3D.resize(n_particles3D);
  dprobvec.resize(n_particles3D);
  sprobvec.resize(n_particles3D);

  // Set particles to zero
  particles3D.resize(n_particles3D);
  for (size_t i = 0; i < particles3D.size(); i++) {
    particles3D[i].id = i;
    particles3D[i].p  = pose(0., 0., 0., 0., 0., 0.);
    particles3D[i].w  = static_cast<float>(1. / n_particles3D);
  }

  // Initialize particles
  p_odom = pose(0., 0., 0.);
  motionModel(initial_pose);

  // Initialize the previous pose
  p_odom = initial_pose;
}
void PF::process(const pose&                         odom,
                 const std::vector<SemanticFeature>& landmarks,
                 const std::vector<ImageFeature>&    features,
                 OccupancyMap&                       grid_map)
{
  // Reset weights sum
  w_sum = 0.;

  // ------------------------------------------------------------------------------
  // ---------------- Draw particles using odometry motion model
  // ------------------------------------------------------------------------------
  motionModel(odom);
  // MISSING ESTIMATION ...
  // ------------------------------------------------------------------------------
  // ---------------- Normalize particle weights using scan matching alignment errors
  // ------------------------------------------------------------------------------
  normalizeWeights();
  // ------------------------------------------------------------------------------
  // ---------------- Resample particles
  // ------------------------------------------------------------------------------
  //  resample(particles2D); // - 2D particles
  resample(particles3D); // - 3D particles

  // - Save current control to use in the next iteration
  p_odom = odom;
}

void PF::motionModel(const pose& odom)
{
  // 3D particle filter
  for (auto& particle : particles3D) {
    drawFromMotion(odom, particle.p);

    // Set unobservable components - draw from zero-mean gaussian distribution
    particle.p.z += sampleGaussian(sigma_z);
    particle.p.roll += sampleGaussian(sigma_roll);
    particle.p.pitch += sampleGaussian(sigma_pitch);
  }
}

void PF::drawFromMotion(const pose& odom, pose& p)
{
  float sxy = static_cast<float>(.3) * srr;

  // Compute delta pose between last and current control
  pose dt_pose = odom - p_odom;
  dt_pose.yaw  = atan2(sin(dt_pose.yaw), cos(dt_pose.yaw));
  float s1 = sin(p_odom.yaw), c1 = cos(p_odom.yaw);
  // ------------------------------------------------------
  pose delta;
  delta.x   = c1 * dt_pose.x + s1 * dt_pose.y;
  delta.y   = -s1 * dt_pose.x + c1 * dt_pose.y;
  delta.yaw = dt_pose.yaw;
  // ------------------------------------------------------
  // Sample zero mean gaussian to compute noisy pose
  pose noisypoint(delta);
  noisypoint.x += sampleGaussian(srr * fabs(delta.x) + str * fabs(delta.yaw) +
                                 sxy * fabs(delta.y));
  noisypoint.y += sampleGaussian(srr * fabs(delta.y) + str * fabs(delta.yaw) +
                                 sxy * fabs(delta.x));
  noisypoint.yaw += sampleGaussian(
      stt * fabs(delta.yaw) + srt * sqrt(delta.x * delta.x + delta.y * delta.y));
  // -------------------------------------------------------------------------------
  noisypoint.normalize();
  // -------------------------------------------------------------------------------

  // Apply motion model
  // ------------------------------------------------------
  float s2 = sin(p.yaw), c2 = cos(p.yaw);
  p.x += c2 * noisypoint.x - s2 * noisypoint.y;
  p.y += s2 * noisypoint.x + c2 * noisypoint.y;
  p.yaw += noisypoint.yaw;
  // ------------------------------------------------------
}

void PF::normalizeWeights()
{
  if (w_sum > 0.) {
    for (auto& particle : particles3D) particle.w /= w_sum;
  } else {
    for (auto& particle : particles3D)
      particle.w = static_cast<float>(1.) / static_cast<float>(particles3D.size());
  }
}

void PF::resample(std::vector<Particle>& particles)
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

void PF::getParticles(std::vector<Particle>& in)
{
  in.resize(particles3D.size());
  for (size_t i = 0; i < particles3D.size(); i++) in[i] = particles3D[i];
}

}; // namespace vineslam
