#pragma once

#include "vineslam_ros.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

#include <thread>

namespace vineslam
{

enum BAG_STATE { PAUSED, PLAYING };

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
  void debugPF(const cv::Mat&                               left_image,
               const sensor_msgs::ImageConstPtr&            depth_image,
               const vision_msgs::Detection2DArrayConstPtr& dets);

  // Input keyboard reader thread to pause and play the bagfile
  void listenStdin();

  // Private replay node objects
  PF* pf;

  // Topic and rosbag names
  std::string bagfile_str;
  std::string odom_str;
  std::string rs_odom_str;
  std::string tf_str;
  std::string fix_str;
  std::string depth_img_str;
  std::string left_img_str;
  std::string pcl_str;

  // System flags
  int       nmessages;
  BAG_STATE bag_state;
};

} // namespace vineslam