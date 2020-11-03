#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/vineslam_report/qnode.hpp"

namespace vineslam_report
{

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
  // ROS services
  rnode_state_srv_client = n.serviceClient<vineslam_ros::change_replay_node_state>(
      "change_replay_node_state");
  rnode_features_srv_client =
      n.serviceClient<vineslam_ros::change_replay_node_features>(
          "change_replay_node_features");
  rnode_debug_pf_srv_client =
      n.serviceClient<vineslam_ros::debug_particle_filter>("debug_pf");

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

void QNode::changeReplayNodeState(const std_msgs::Bool& pause,
                                  const std_msgs::Bool& play,
                                  const std_msgs::Bool& iterate)
{
  vineslam_ros::change_replay_node_state srv;
  srv.request.pause_node   = pause;
  srv.request.play_node    = play;
  srv.request.iterate_node = iterate;

  rnode_state_srv_client.call(srv);
}

void QNode::changeReplayNodeFeatures(const std_msgs::Bool& use_high_level,
                                     const std_msgs::Bool& use_corners,
                                     const std_msgs::Bool& use_planars,
                                     const std_msgs::Bool& use_planes,
                                     const std_msgs::Bool& use_ground,
                                     const std_msgs::Bool& use_image_features,
                                     const std_msgs::Bool& use_gps)
{
  vineslam_ros::change_replay_node_features srv;
  srv.request.use_high_level = use_high_level;
  srv.request.use_corners    = use_corners;
  srv.request.use_planars    = use_planars;
  srv.request.use_planes     = use_planes;
  srv.request.use_ground     = use_ground;
  srv.request.use_icp        = use_image_features;
  srv.request.use_gps        = use_gps;

  rnode_features_srv_client.call(srv);
}

void QNode::callParticleFilterDebugger(const float& x_std,
                                       const float& y_std,
                                       const float& z_std,
                                       const float& R_std,
                                       const float& P_std,
                                       const float& Y_std)
{
  vineslam_ros::debug_particle_filter srv;
  srv.request.x_std = x_std;
  srv.request.y_std = y_std;
  srv.request.z_std = z_std;
  srv.request.R_std = R_std;
  srv.request.P_std = P_std;
  srv.request.Y_std = Y_std;

  rnode_debug_pf_srv_client.call(srv);
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
