#pragma once

#ifndef Q_MOC_RUN
#include "../../utils/qnode.hpp"
#include <ros/ros.h>
#endif
#include "landmark_processor.hpp"
#include "mapper.hpp"
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/features2d.hpp>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

/* edgetpu detection API */
#include <detection/engine.h>
#include <examples/label_utils.h>
#include <examples/model_utils.h>
#include <test_utils.h>

class Mapper : public QNode
{
	Q_OBJECT

public:
	Mapper(int argc, char** argv);
	virtual ~Mapper() {}
	void run();
	void rosCommsInit();
	void retrieveLog(std::string& log);
	void retrieveLog(std::string& log, const int& id);
	void constructMap(const float& scaler, const int& map_width,
	                  const int& map_heigth);

  const cv::Mat filterMap();
	const cv::Mat exportMap();
	const cv::Mat exportHistogram();
	const cv::Mat exportSingleMap(const int& id, const float& scaler,
	                              const int& map_width, const int& map_heigth);

	void saveMap();
	bool histogramType();

Q_SIGNALS:
	void init_done();

private:
	void imageListener(const sensor_msgs::ImageConstPtr& msg);
	void poseListener(const geometry_msgs::PoseStampedConstPtr& msg);
#ifdef DEBUG
	void showMatching(cv::Mat img);
	void showBBoxes(const sensor_msgs::ImageConstPtr& msg, cv::Mat& bboxes,
	                std::vector<coral::DetectionCandidate> res);

	cv::Mat p_image;
	cv::Mat c_image;

	image_transport::Publisher matches_publisher;
#endif

	ros::Subscriber img_subscriber;
	ros::Subscriber pose_subscriber;

	image_transport::Publisher img_publisher;

	bool init;

	Pose<double>              last_pose;
	std::vector<Pose<double>> all_poses;

	std_msgs::Header    scan_header;
	geometry_msgs::Pose slam_pose;

	Estimator*         estimator;
	LandmarkProcessor* lprocessor;
	Parameters*        params;

	std::vector<int>                     input_tensor_shape;
	coral::DetectionEngine*              engine;
	std::unordered_map<int, std::string> labels;

	void loadParameters(const ros::NodeHandle& local_nh)
	{
		/* read launch file parameters */
		local_nh.getParam("/mapper/pose_topic", (*params).pose_topic);
		local_nh.getParam("/mapper/image_topic", (*params).image_topic);
		local_nh.getParam("/mapper/h_fov", (*params).h_fov);
		local_nh.getParam("/mapper/v_fov", (*params).v_fov);
		local_nh.getParam("/mapper/img_width", (*params).width);
		local_nh.getParam("/mapper/img_height", (*params).height);
		local_nh.getParam("/mapper/match_box", (*params).match_box);
		local_nh.getParam("/mapper/filter_window", (*params).filter_window);
		local_nh.getParam("/mapper/mapper_inc", (*params).mapper_inc);
		local_nh.getParam("/mapper/detector_th", (*params).min_score);
		local_nh.getParam("/mapper/max_stdev", (*params).max_stdev);
		local_nh.getParam("/mapper/model_path", (*params).model);
		local_nh.getParam("/mapper/labels_path", (*params).labels);
		local_nh.getParam("/mapper/prediction", (*params).prediction);
		local_nh.getParam("/mapper/vineyard_height", (*params).vineyard_height);
	}
};
