#pragma once

#include "vineslam_ros.hpp"
#include <vineslam/math/stat.hpp>
#include <vineslam_ros/change_replay_node_state.h>
#include <vineslam_ros/change_replay_node_features.h>
#include <vineslam_ros/debug_particle_filter.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

#include <thread>

namespace vineslam
{
enum BAG_STATE
{
  PAUSED,
  PLAYING,
  ITERATING
};

class ReplayNode : public VineSLAM_ros
{
public:
  // Class constructor that
  // - Initialize the ROS node
  // - Define the publish and subscribe topics
  ReplayNode(int argc, char** argv);

  // Class destructor - saves the map to an output xml file
  ~ReplayNode();

private:
  // Bag file iterator function - for offline mode
  void replayFct(ros::NodeHandle nh);

  // Debug particle filter
  bool debugPF(vineslam_ros::debug_particle_filter::Request& request,
               vineslam_ros::debug_particle_filter::Response& response);

  // Node services
  bool changeNodeState(vineslam_ros::change_replay_node_state::Request&,
                       vineslam_ros::change_replay_node_state::Response&);
  bool changeNodeFeatures(vineslam_ros::change_replay_node_features::Request&,
                          vineslam_ros::change_replay_node_features::Response&);

  // Input keyboard reader thread to pause and play the bagfile
  void listenStdin();

  // Private replay node objects
  PF* debug_pf_;
  OccupancyMap* debug_grid_map_{};
  std::vector<Particle> debug_particles_vec_;

  // Replay node ROS publishers
  ros::Publisher debug_pf_particles_pub_;
  ros::Publisher debug_pf_weights_pub_;
  ros::Publisher debug_pf_corners_local_pub_;
  ros::Publisher debug_pf_planars_local_pub_;
  ros::Publisher debug_pf_planes_local_pub_;
  ros::Publisher debug_pf_ground_local_pub_;

  // System flags
  int nmessages_;
  BAG_STATE bag_state_;
  bool have_iterated_{};
};

}  // namespace vineslam