#include "localizer.hpp"

namespace vineslam
{

Localizer::Localizer(const std::string& config_path)
    : config_path(config_path)
{
  // Read input parameters
  YAML::Node config = YAML::LoadFile(config_path);
  img_width         = config["camera_info"]["img_width"].as<float>();
  img_height        = config["camera_info"]["img_height"].as<float>();
  cam_height        = config["camera_info"]["cam_height"].as<float>();
  fx                = config["camera_info"]["fx"].as<float>();
  fy                = config["camera_info"]["fy"].as<float>();
  cx                = config["camera_info"]["cx"].as<float>();
  cy                = config["camera_info"]["cy"].as<float>();
  n_particles       = config["pf"]["n_particles"].as<int>();
  use_icp           = config["pf"]["use_icp"].as<bool>();
  k_clusters        = config["pf"]["k_clusters"].as<int>();
  num_threads       = config["system"]["num_threads"].as<int>();

  it = 0;
}

void Localizer::init(const pose& initial_pose)
{
  // Certificate that the number of threads and number of particles are multiples
  if (n_particles % num_threads != 0) {
    n_particles = num_threads * static_cast<int>(n_particles / num_threads);
  }
  // Initialize the particle filter
  pf = new PF(config_path, initial_pose, n_particles);

  // Compute average pose and standard deviation of the
  // first distribution
  std::vector<pose> poses;
  for (auto& particle : pf->particles) poses.push_back(particle.p);
  average_pose = pose(poses);
}

void Localizer::process(const pose&         odom,
                        const Observation&  obsv,
                        const OccupancyMap& grid_map)
{
  auto before = std::chrono::high_resolution_clock::now();
  // Reset weights sum
  pf->w_sum = 0.;

  // ------------------------------------------------------------------------------
  // ---------------- Draw particles using odometry motion model
  // ------------------------------------------------------------------------------
  pf->motionModel(odom);
  // ------------------------------------------------------------------------------
  // ---------------- Update particles weights using multi-layer map
  // ------------------------------------------------------------------------------
  if (num_threads == 1) { // single thread
    pf->update(0,
               n_particles,
               obsv.landmarks,
               obsv.corners,
               obsv.ground_plane,
               obsv.gps_pose,
               grid_map);

  } else { // Multi threading
    // Trough threads
    int xmin, xmax;

    xmin = 0 * (n_particles / num_threads);
    xmax = xmin + (n_particles / num_threads);
    std::thread t1(&PF::update,
                   pf,
                   xmin,
                   xmax,
                   obsv.landmarks,
                   obsv.corners,
                   obsv.ground_plane,
                   obsv.gps_pose,
                   grid_map);
    xmin = 1 * (n_particles / num_threads);
    xmax = xmin + (n_particles / num_threads);
    std::thread t2(&PF::update,
                   pf,
                   xmin,
                   xmax,
                   obsv.landmarks,
                   obsv.corners,
                   obsv.ground_plane,
                   obsv.gps_pose,
                   grid_map);
    xmin = 2 * (n_particles / num_threads);
    xmax = xmin + (n_particles / num_threads);
    std::thread t3(&PF::update,
                   pf,
                   xmin,
                   xmax,
                   obsv.landmarks,
                   obsv.corners,
                   obsv.ground_plane,
                   obsv.gps_pose,
                   grid_map);
    xmin = 3 * (n_particles / num_threads);
    xmax = xmin + (n_particles / num_threads);
    std::thread t4(&PF::update,
                   pf,
                   xmin,
                   xmax,
                   obsv.landmarks,
                   obsv.corners,
                   obsv.ground_plane,
                   obsv.gps_pose,
                   grid_map);

    t1.join();
    t2.join();
    t3.join();
    t4.join();
  }

  // ------------------------------------------------------------------------------
  // ---------------- Normalize particle weights
  // ------------------------------------------------------------------------------
  pf->normalizeWeights();

  if (use_icp) {
    // ------------------------------------------------------------------------------
    // ---------------- Cluster particles
    // ------------------------------------------------------------------------------
    std::map<int, Gaussian<pose, pose>> gauss_map;
    pf->cluster(gauss_map);
#ifdef TEST
    saveParticleClusters(gauss_map, pf->particles, k_clusters, it);
#endif

    // ------------------------------------------------------------------------------
    // ---------------- Scan match
    // ------------------------------------------------------------------------------
    pf->scanMatch(gauss_map, obsv.surf_features, grid_map);
  }
  // ------------------------------------------------------------------------------
  // ---------------- Resample particles
  // ------------------------------------------------------------------------------
  pf->resample();

  // - Compute final robot pose using the mean of the particles poses
  std::vector<pose> poses;
  for (const auto& particle : pf->particles) poses.push_back(particle.p);
  average_pose = pose(poses);

  it++;

  // - Save current control to use in the next iteration
  pf->p_odom = odom;
  auto after = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float, std::milli> duration = after - before;
  std::cout << "Time elapsed on PF (msecs): " << duration.count() << std::endl;
}

pose Localizer::getPose() const { return average_pose; }

void Localizer::getParticles(std::vector<pose>& in) const
{
  in.resize(pf->particles.size());
  for (size_t i = 0; i < in.size(); i++) in[i] = pf->particles[i].p;
}

} // namespace vineslam
