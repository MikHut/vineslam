/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/vineslam_report/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace vineslam_report
{

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv)
    : init_argc(argc)
    , init_argv(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ROS_INFO("Initializing QNode");
  ros::init(init_argc, init_argv, "vineslam_report");
  if (!ros::master::check()) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // ROS subscriptions
  report_sub = n.subscribe("/vineslam/report", 1, &QNode::reportSubscriber, this);

  start();
  return true;
}

void QNode::run()
{
  ros::spin();
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT
  rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::reportSubscriber(const vineslam_msgs::reportConstPtr& msg)
{
  debugger.setReport(*msg);

  cv::Mat bx_hist, by_hist, bz_hist, ax_hist, ay_hist, az_hist;
  cv::Mat bR_hist, bP_hist, bY_hist, aR_hist, aP_hist, aY_hist;

  debugger.plotXYZHists(bx_hist, by_hist, bz_hist, ax_hist, ay_hist, az_hist);
  debugger.plotRPYHists(bR_hist, bP_hist, bY_hist, aR_hist, aP_hist, aY_hist);

  drawHistOnGui(bx_hist,
                by_hist,
                bz_hist,
                ax_hist,
                ay_hist,
                az_hist,
                bR_hist,
                bP_hist,
                bY_hist,
                aR_hist,
                aP_hist,
                aY_hist);
}

void QNode::log(const LogLevel& level, const std::string& msg)
{
  logging_model.insertRows(logging_model.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level) {
    case (Debug): {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Info): {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Warn): {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Error): {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Fatal): {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1), new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

} // namespace vineslam_report
