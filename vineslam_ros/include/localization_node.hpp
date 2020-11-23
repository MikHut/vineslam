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
};

}  // namespace vineslam