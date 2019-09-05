#pragma once

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <landmark_processor.hpp>
#include <ros/ros.h>

class Odometer
{
public:
  Odometer();
  void boxListener(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);

private:
  int width;
  int heigth;
  int resolution;

  LandmarkProcessor *processor;
};
