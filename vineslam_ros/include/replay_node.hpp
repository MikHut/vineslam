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

  bool changeNodeState(vineslam_ros::change_replay_node_state::Request&,
                       vineslam_ros::change_replay_node_state::Response&);

  // Input keyboard reader thread to pause and play the bagfile
  void listenStdin();

  // Standalone node parameters
  std::string image_topic_, depth_topic_;

  // System flags
  int nmessages_;
  BAG_STATE bag_state_;
  bool have_iterated_{};
};

}  // namespace vineslam