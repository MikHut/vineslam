#pragma once

#include "vineslam_ros.hpp"
#include <vineslam/matcher/icp.hpp>

namespace vineslam
{
class LocalizationNode : public VineSLAM_ros
{
public:
  // Class constructor that
  // - Initialize the ROS node
  // - Define the publish and subscribe topics
  LocalizationNode(int argc, char** argv);

  // Class destructor - saves the map to an output xml file
  ~LocalizationNode();

private:
  // Parameters loader
  void loadParameters(const ros::NodeHandle& nh, Parameters& params);

  // Runtime execution routines
  void init();
  void loop();
  void loopOnce();
  void process();

  // Thread to publish the tfs exported by the localization node
  // NOTE: We perform this process in a thread since we both need to do it on runtime and when setting the initial pose
  //       In the second case, if not in a thread, the tfs are only published when the used sends some feedback through
  //       the interactive marker.
  void broadcastTfs();

  // Routine to set the initial 6-DoF pose of the robot in relation with the previously built map
  void initializeOnMap();
  // Interactive marker callback functions
  void iMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void iMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  // Interactive marker for initialization variables
  interactive_markers::MenuHandler im_menu_handler_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;

  // ROS subscribers
  ros::Subscriber scan_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber gps_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber imu_data_subscriber_;

  // ROS services
  ros::ServiceServer save_map_srv_;
};

}  // namespace vineslam