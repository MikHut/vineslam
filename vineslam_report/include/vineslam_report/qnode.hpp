#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/network.h>
#endif
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <vineslam_msgs/report.h>
#include "debugger.hpp"

namespace vineslam_report
{

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  // Logging
  enum LogLevel { Debug, Info, Warn, Error, Fatal };

  QStringListModel* loggingModel() { return &logging_model; }
  void              log(const LogLevel& level, const std::string& msg);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void drawHistOnGui(cv::Mat bx_hist,
                     cv::Mat by_hist,
                     cv::Mat bz_hist,
                     cv::Mat ax_hist,
                     cv::Mat ay_hist,
                     cv::Mat az_hist,
                     cv::Mat bR_hist,
                     cv::Mat bP_hist,
                     cv::Mat bY_hist,
                     cv::Mat aR_hist,
                     cv::Mat aP_hist,
                     cv::Mat aY_hist);

private:
  void reportSubscriber(const vineslam_msgs::reportConstPtr& msg);

  ros::Subscriber  report_sub;
  Debugger         debugger;
  int              init_argc;
  char**           init_argv;
  QStringListModel logging_model;
};

} // namespace vineslam_report
