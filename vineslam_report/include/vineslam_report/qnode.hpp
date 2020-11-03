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

#include <vineslam_ros/change_replay_node_state.h>
#include <vineslam_ros/change_replay_node_features.h>
#include <vineslam_ros/debug_particle_filter.h>
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

  void changeReplayNodeState(const std_msgs::Bool& pause,
                             const std_msgs::Bool& play,
                             const std_msgs::Bool& iterate);
  void changeReplayNodeFeatures(const std_msgs::Bool& use_high_level,
                                const std_msgs::Bool& use_corners,
                                const std_msgs::Bool& use_planars,
                                const std_msgs::Bool& use_planes,
                                const std_msgs::Bool& use_ground,
                                const std_msgs::Bool& use_image_features,
                                const std_msgs::Bool& use_gps);
  void callParticleFilterDebugger(const float& x_std,
                                  const float& y_std,
                                  const float& z_std,
                                  const float& R_std,
                                  const float& P_std,
                                  const float& Y_std);

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

  ros::Subscriber    report_sub;
  ros::ServiceClient rnode_state_srv_client;
  ros::ServiceClient rnode_features_srv_client;
  ros::ServiceClient rnode_debug_pf_srv_client;
  Debugger           debugger;
  int                init_argc;
  char**             init_argv;
  QStringListModel   logging_model;
};

} // namespace vineslam_report
