#include "pf.hpp"

namespace wildSLAM
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
  particles2D.resize(n_particles2D);
  particles3D.resize(n_particles3D);
  for (size_t i = 0; i < particles2D.size(); i++) {
    particles2D[i].id = i;
    particles2D[i].p  = pose(0., 0., 0., 0., 0., 0.);
    particles2D[i].w  = static_cast<float>(1. / n_particles2D);
  }
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
void PF::process(const pose&                  odom,
                 const std::vector<Landmark>& landmarks,
                 const std::vector<Feature>&  features,
                 OccupancyMap&                grid_map)
{
  // Reset weights sum
  w_sum = 0.;

  // ------------------------------------------------------------------------------
  // ---------------- Draw particles using odometry motion model
  // ------------------------------------------------------------------------------
  motionModel(odom);
  // ------------------------------------------------------------------------------
  // ---------------- Perform scan matching using the odometry motion model as first
  // ---------------- guess for each particle & Update particle weights
  // ------------------------------------------------------------------------------
  auto before = std::chrono::high_resolution_clock::now();
  scanMatch(features, grid_map);
  auto after = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float, std::milli> duration = after - before;
  std::cout << "Time elapsed (msecs): " << duration.count() << std::endl;
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
  // 2D particle filter
  for (auto& particle : particles2D) {
    drawFromMotion(odom, particle.p);
  }

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

void PF::scanMatch(const std::vector<Feature>& features, OccupancyMap& grid_map)
{
  // -------------------------------------------------------------------------------
  // ------- 3D scan matching using low level features
  // -------------------------------------------------------------------------------
  // - Set target point cloud (common to all particles)
  icp->setInputTarget(grid_map);
  // - Perform scan matching for each particle
  for (size_t i = 0; i < particles3D.size(); i++) {
    // Convert particle i pose to [R|t]
    std::array<float, 3> trans = {
        particles3D[i].p.x, particles3D[i].p.y, particles3D[i].p.z};
    std::array<float, 9> Rot = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
    particles3D[i].p.toRotMatrix(Rot);

    // --------------- Perform scan matching ---------------------
    // - First guess: each particle drawn by odometry motion model
    icp->setInputSource(features);
    std::vector<Feature> aligned;
    float                rms_error;
    // - Only use scan match if it does no fail
    if (icp->align(Rot, trans, rms_error, aligned)) {
      // - Get homogeneous transformation result
      std::array<float, 9> final_Rot   = {0., 0., 0., 0., 0., 0., 0., 0., 0.};
      std::array<float, 3> final_trans = {0., 0., 0.};
      icp->getTransform(final_Rot, final_trans);

      // - Convert homogeneous transformation to translation and Euler angles
      // and set the particle pose
      particles3D[i].p = pose(final_Rot, final_trans);

      // - Save scan matcher spatial and descriptor gaussian distribution
      Gaussian<float, float> sprob{}, dprob{};
      icp->getProb(sprob, dprob);

      // - Get the correspondences errors both spatial and for the descriptors
      std::vector<float> serror;
      std::vector<float> derror;
      icp->getErrors(serror, derror);

      // - Prevent single correspondence - standard deviation = 0
      if (serror.size() <= 1) {
        particles3D[i].w = 0.;
        continue;
      }

      // ---------- Update particle weight ----------- //
      float w = 0.;
      for (size_t j = 0; j < serror.size(); j++) {
        auto m_sprob = static_cast<float>(
            (1. / (std::sqrt(2. * PI) * sprob.stdev)) *
            exp(-serror[j] / (2. * PI * sprob.stdev * sprob.stdev)));
        auto m_dprob = static_cast<float>(
            (1. / (std::sqrt(2. * PI) * dprob.stdev)) *
            exp(-derror[j] / (2. * PI * dprob.stdev * dprob.stdev)));

        w += m_sprob * m_dprob;
      }
      particles3D[i].w = w;
      w_sum += w;
    } else {
      // - If scan match fails, we set the particle weight to zero
      particles3D[i].w = 0.;
    }
  }
}

void PF::normalizeWeights()
{
  if (w_sum > 0.) {
    for (auto& particle : particles3D) particle.w /= w_sum;
  } else {
    for (auto& particle : particles3D)
      particle.w = static_cast<float>(1.) / static_cast<float>(particles3D.size());
  }

  std::cout << "BEFORE RESAMPLING " << std::endl;
  for (const auto& p : particles3D)
    std::cout << "Particle i: " << p.w << " -> " << p.p;
  std::cout << std::endl;
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
    std::cout << j << " - " << particles[j].w << std::endl;
    particles[j].p = particles[indexes[j]].p;
    particles[j].w = particles[indexes[j]].w;
    std::cout << indexes[j] << " - " << particles[indexes[j]].w << std::endl;
  }
  std::cout << std::endl;
}

void PF::getParticles2D(std::vector<Particle>& in)
{
  in.resize(particles2D.size());
  for (size_t i = 0; i < particles2D.size(); i++) in[i] = particles2D[i];
}

void PF::getParticles3D(std::vector<Particle>& in)
{
  in.resize(particles3D.size());
  for (size_t i = 0; i < particles3D.size(); i++) in[i] = particles3D[i];
}

}; // namespace wildSLAM
