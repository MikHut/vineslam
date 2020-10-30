#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <vineslam_msgs/report.h>
#include <math.h>

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
  void plotXYZHists(cv::Mat& bx_hist,
                    cv::Mat& by_hist,
                    cv::Mat& bz_hist,
                    cv::Mat& ax_hist,
                    cv::Mat& ay_hist,
                    cv::Mat& az_hist);

  void plotRPYHists(cv::Mat& bR_hist,
                    cv::Mat& bP_hist,
                    cv::Mat& bY_hist,
                    cv::Mat& aR_hist,
                    cv::Mat& aP_hist,
                    cv::Mat& aY_hist);

private:
  vineslam_msgs::report m_report;

  // Auxiliary functions
  template <typename T>
  std::string to_string_with_precision(const T a_value, const int n = 3)
  {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
  }
};