#include "../include/odometer.hpp"

Odometer::Odometer()
{
  ros::NodeHandle local_nh("~");

  /* read launch file parameters */
  local_nh.getParam("/odometer/img_width", width);
  local_nh.getParam("/odometer/img_height", heigth);
  local_nh.getParam("/odometer/grid_resolution", resolution);

  processor = new LandmarkProcessor(width, heigth, resolution);
}

void Odometer::boxListener(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
  /* center of mass calculation */
  std::vector<Point<int>> center_of_mass;
  for (auto i : (*msg).bounding_boxes) {
    Point<int> tmp((i.xmin + i.xmax) / 2, (i.ymin + i.ymax) / 2);
    center_of_mass.push_back(tmp);
  }

  (*processor).updatePoses(center_of_mass);
  cv::Mat grid = (*processor).buildGrid();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometer");

  ros::NodeHandle nh;

  Odometer*       odometer = new Odometer();
  ros::Subscriber box_sub  = nh.subscribe("/darknet_ros/bounding_boxes", 1000,
					  &Odometer::boxListener, odometer);

  ros::spin();
  return 0;
}
