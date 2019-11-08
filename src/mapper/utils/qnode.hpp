#pragma once

#ifndef Q_MOC_RUN
#include <ros/network.h>
#include <ros/ros.h>
#endif
#include <QStringListModel>
#include <QThread>
#include <geometry_msgs/Twist.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/Int64.h>
#include <string>

/*!
 * - This class initializes and finishes ROS connection
 */

class QNode : public QThread
{
	Q_OBJECT

public:
	/*! - class constructor */
	QNode(int argc, char** argv, const std::string& node_name);
	/*! - class destructor, shutdowns ROS */
	virtual ~QNode();

	/*! - initializes the ROS connection using the class contructor parameters */
	bool init();

	virtual void          run()                                        = 0;
	virtual void          retrieveLog(std::string& log)                = 0;
	virtual void          retrieveLog(std::string& log, const int& id) = 0;
	virtual void          constructMap()                               = 0;
	virtual const cv::Mat exportMap()                                  = 0;
	virtual const cv::Mat exportSingleMap(const int& id)               = 0;

	const std::string node_name;

Q_SIGNALS:
	/*! - signals ROS termination */
	void rosShutdown();

protected:
	int    init_argc;
	char** init_argv;

	virtual void rosCommsInit() = 0;
};
