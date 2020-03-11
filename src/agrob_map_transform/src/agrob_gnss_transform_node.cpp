#include "agrob_map_transform/agrob_gnss_transform.h"
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "agrob_transform_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  RobotLocalization::NavSatTransform trans(nh, nh_priv);
  ros::spin();
  return EXIT_SUCCESS;
}


