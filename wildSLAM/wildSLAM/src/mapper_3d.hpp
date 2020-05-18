#pragma once

#include <iostream>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <yaml-cpp/yaml.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Math
#include <math/point3D.hpp>
#include <math/pose6D.hpp>

// Classes
#include <feature.hpp>

#define PI 3.14159265359

class Mapper3D
{
public:
  // Class constructor - receives and saves the system
  // parameters
  explicit Mapper3D(const std::string& config_path);

private:
  // Camera info parameters
  float img_width;
  float img_height;
  float cam_height;
  float fx;
  float fy;
  float cx;
  float cy;
};
