#pragma once

#include "vineslam_ros.hpp"

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
  // Initialization function. Initializes the occupancy grid map and the localization system.
  void init();
   // Loop function. Calls the loopOnce() function during all the execution.
  void loop();
  // Step function. Calls the process() function during all the execution.
  void loopOnce();
  // Main node function. Processes the observations and calls the map builders
  void process();

};

}  // namespace vineslam