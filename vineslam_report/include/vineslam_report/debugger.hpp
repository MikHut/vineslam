#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <vineslam_msgs/report.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <QImage>

class Debugger
{
public:
  Debugger() = default;

  // Setter functions
  void setReport(const vineslam_msgs::report& report);

  // Histogram plotters
  cv::Mat plotXYZHists();
  cv::Mat plotRPYHists();

private:
  vineslam_msgs::report m_report;
};