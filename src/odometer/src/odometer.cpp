#include "../include/odometer.hpp"

Odometer::Odometer()
{
	ros::NodeHandle local_nh("~");

	params = new Parameters();
	loadParameters(local_nh);

#ifdef VISUALIZE
	p_image = cv::Mat((*params).width, (*params).height, CV_8UC1,
	                  cv::Scalar(255, 255, 255));
	c_image = p_image;
#endif

	pfilter    = new ParticleFilter(*params);
	lprocessor = (*pfilter).landm_obj;

	(*pfilter).init();
	init = true;
}

/* void Odometer::poseListener(const nav_msgs::OdometryConstPtr& msg) */
void Odometer::poseListener(const geometry_msgs::PoseStampedConstPtr& msg)
{
	slam_pose = (*msg).pose;
}

void Odometer::boxListener(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
	tf::Pose pose;
	tf::poseMsgToTF(slam_pose, pose);
	double yaw = tf::getYaw(pose.getRotation());

	Pose<double> delta_pose;
	if (init == false) {
		Point<double> pt((slam_pose.position.x - last_pose.pos.x) * 100,
		                 (slam_pose.position.y - last_pose.pos.y) * 100);
		delta_pose = Pose<double>(pt, yaw - last_pose.theta);
	}
	else {
		init       = false;
		delta_pose = Pose<double>(Point<double>(MEAN_X, MEAN_Y), MEAN_THETA);
	}

	/* trunks center of mass calculation */
	std::vector<Point<double>> trunk_pos;
	for (auto i : (*msg).bounding_boxes) {
		Point<double> tmp((i.xmin + i.xmax) / 2, (i.ymin + i.ymax) / 2);
		trunk_pos.push_back(tmp);
	}

	if (delta_pose.pos.x > 0.0 && delta_pose.pos.y > 0.0)
		(*pfilter).process(trunk_pos, delta_pose);

#ifdef VISUALIZE /* PointCloud publisher */
	sensor_msgs::PointCloud pcl;
	for (size_t i = 0; i < (*pfilter).landmarks.size(); i++) {
		geometry_msgs::Point32 m_pos;
		m_pos.x = (*pfilter).landmarks[i].world_pos.x / 100;
		m_pos.y = (*pfilter).landmarks[i].world_pos.y / 100;

		pcl.header = (*msg).header;
		pcl.points.push_back(m_pos);
	}
	pcl_pub.publish(pcl);
#endif

	last_pose.pos.x = slam_pose.position.x;
	last_pose.pos.y = slam_pose.position.y;
	last_pose.theta = yaw;

#ifdef VISUALIZE
	/* visual grid publication */
	sensor_msgs::ImagePtr grid_img =
	    cv_bridge::CvImage(std_msgs::Header(), "mono8", (*lprocessor).p_map)
	        .toImageMsg();
	grid_pub.publish(grid_img);

	/* particles distribution visualization */
	sensor_msgs::ImagePtr p_img =
	    cv_bridge::CvImage(std_msgs::Header(), "mono8", (*pfilter).box)
	        .toImageMsg();
	particles_pub.publish(p_img);
#endif
}

void Odometer::imageListener(const sensor_msgs::ImageConstPtr& msg)
{
	p_image = c_image;
	c_image =
	    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;

	std::vector<cv::DMatch>   m;
	std::vector<cv::KeyPoint> keyPointRight;
	std::vector<cv::KeyPoint> keyPointLeft;
	std::vector<cv::Point2f>  vecRight;
	std::vector<cv::Point2f>  vecLeft;

	for (size_t i = 0; i < (*lprocessor).matches.size(); i++) {
		vecRight.push_back(cv::Point2f((*lprocessor).matches[i].c_pos.x,
		                               (*lprocessor).matches[i].c_pos.y));
		vecLeft.push_back(cv::Point2f((*lprocessor).matches[i].p_pos.x,
		                              (*lprocessor).matches[i].p_pos.y));
	}
	cv::KeyPoint::convert(vecRight, keyPointRight);
	cv::KeyPoint::convert(vecLeft, keyPointLeft);

	for (size_t j = 0; j < vecRight.size(); j++)
		m.push_back(cv::DMatch(j, j, 0));

	cv::Mat img_matches;
	cv::drawMatches(p_image, keyPointLeft, c_image, keyPointRight, m,
	                img_matches);

	sensor_msgs::ImagePtr img =
	    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_matches).toImageMsg();
	matches_pub.publish(img);
}
