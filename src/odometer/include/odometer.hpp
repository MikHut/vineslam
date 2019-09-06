#pragma once

#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <image_transport/image_transport.h>
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

  cv::Mat last_grid;

  image_transport::Publisher grid_pub;

  LandmarkProcessor* processor;
};
