#include "localizer.hpp"

#include <utility>

namespace vineslam
{

Localizer::Localizer(Parameters params)
    : params(std::move(params))
{
}

void Localizer::init(const pose& initial_pose)
{
  // Initialize the particle filter
  pf = new PF(params, initial_pose);

  // Compute average pose and standard deviation of the
  // first distribution
  std::vector<pose> poses;
  for (auto& particle : pf->particles) poses.push_back(particle.p);
  average_pose     = pose(poses);
  last_update_pose = initial_pose;

  init_flag = true;
}

void Localizer::process(const pose&        odom,
                        const Observation& obsv,
                        OccupancyMap*      previous_map,
                        OccupancyMap*      grid_map)
{
  auto before = std::chrono::high_resolution_clock::now();
  // Resets
  pf->w_sum = 0.;
  update    = false;

  // ------------------------------------------------------------------------------
  // ---------------- Draw particles using odometry motion model
  // ------------------------------------------------------------------------------
  pf->motionModel(odom);
  // - Save not resampled particles
  m_particles.clear();
  for (const auto& particle : pf->particles) m_particles.push_back(particle);

  pose delta_pose = odom - last_update_pose;
  delta_pose.normalize();

  if (std::fabs(delta_pose.x) > 0.1 || std::fabs(delta_pose.y) > 0.1 ||
      std::fabs(delta_pose.yaw) > 5 * DEGREE_TO_RAD || init_flag) {

    // ------------------------------------------------------------------------------
    // ---------------- Update particles weights using multi-layer map
    // ------------------------------------------------------------------------------
    pf->update(obsv.landmarks,
               obsv.corners,
               obsv.planars,
               obsv.planes,
               obsv.ground_plane,
               obsv.surf_features,
               obsv.gps_pose,
               previous_map,
               grid_map);

    // ------------------------------------------------------------------------------
    // ---------------- Normalize particle weights
    // ------------------------------------------------------------------------------
    pf->normalizeWeights();

    // ------------------------------------------------------------------------------
    // ---------------- Resample particles
    // ------------------------------------------------------------------------------
    pf->resample();

    last_update_pose = odom;
    init_flag        = false;
    update           = true;
  }

  // - Compute final robot pose using the mean of the particles poses
  std::vector<pose> poses;
  for (const auto& particle : pf->particles) poses.push_back(particle.p);
  average_pose = pose(poses);

  // - Save current control to use in the next iteration
  pf->p_odom = odom;

  // - Save pf logs
  auto after = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float, std::milli> duration = after - before;

  logs = pf->logs +
         "Time elapsed on PF (msecs): " + std::to_string(duration.count()) + "\n\n";
}

pose Localizer::getPose() const { return average_pose; }

void Localizer::getParticles(std::vector<Particle>& in) const
{
  in.resize(pf->particles.size());
  for (size_t i = 0; i < in.size(); i++) in[i] = pf->particles[i];
}

void Localizer::getParticlesBeforeResampling(std::vector<Particle>& in) const
{
  in.resize(m_particles.size());
  for (size_t i = 0; i < in.size(); i++) in[i] = m_particles[i];
}

void Localizer::changeObservationsToUse(const bool& use_high_level,
                                        const bool& use_corners,
                                        const bool& use_planars,
                                        const bool& use_planes,
                                        const bool& use_ground,
                                        const bool& use_icp,
                                        const bool& use_gps)
{
  pf->use_landmarks    = use_high_level;
  pf->use_corners      = use_corners;
  pf->use_planars      = use_planars;
  pf->use_planes       = use_planes;
  pf->use_ground_plane = use_ground;
  pf->use_icp          = use_icp;
  pf->use_gps          = use_gps;
}

void Localizer::predictMotion(const pose&                odom,
                              const std::vector<Corner>& corners,
                              const std::vector<Planar>& planars,
                              OccupancyMap*              previous_map,
                              TF&                        result)
{
  // -------------------------------------------------------------------------------
  // ----- Planar features ICP
  // -------------------------------------------------------------------------------

  // - Set ICP input parameters
  ICP<Planar> planar_icp(params);
  planar_icp.setInputSource(planars);
  planar_icp.setInputTarget(previous_map);
  planar_icp.setMaxIterations(50);
  planar_icp.setRejectOutliersFlag(false);

  // - Convert odom increment pose to TF
  pose odom_inc = odom - pf->p_odom;
  odom_inc.normalize();
  std::array<float, 9> odom_R{};
  odom_inc.toRotMatrix(odom_R);
  TF odom_tf(odom_R, std::array<float, 3>{odom_inc.x, odom_inc.y, odom_inc.z});

  // - Compute ICP
  float               rms_error;
  std::vector<Planar> aligned;
  planar_icp.align(rms_error, aligned);
  //  planar_icp.align(odom_tf, rms_error, aligned);
  planar_icp.getTransform(result);
  pose m_res(result.R, result.t);

  std::vector<float> errors;
  planar_icp.getErrors(errors);

  std::cout << "---- ICP ---- \n\n";
  std::cout << "-> (initial guess): " << odom_inc;
  std::cout << "-> (rms): " << rms_error << "\n";
  std::cout << "-> (result): " << m_res << "---\n";
  pf->p_odom = odom;
}

} // namespace vineslam
