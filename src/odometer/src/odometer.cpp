#include "../include/odometer.hpp"

Odometer::Odometer()
{
	ros::NodeHandle local_nh("~");

	params = new Parameters();
	loadParameters(local_nh);

#ifdef VISUALIZE
	last_grid = cv::Mat((*params).resolution * 100, (*params).resolution * 100,
	                    CV_8UC1, cv::Scalar(255, 255, 255));
	p_image   = cv::Mat((*params).width, (*params).height, CV_8UC1,
                    cv::Scalar(255, 255, 255));
	c_image   = p_image;
#endif

	pfilter    = new ParticleFilter(*params);
	lprocessor = (*pfilter).landm_obj;

	(*pfilter).init();
}

void Odometer::boxListener(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
	/* center of mass calculation */
	std::vector<Point<double>> center_of_mass;
	for (auto i : (*msg).bounding_boxes) {
		Point<double> tmp((i.xmin + i.xmax) / 2, (i.ymin + i.ymax) / 2);
		center_of_mass.push_back(tmp);
	}

	/* compute particle filter estimator */
	(*pfilter).process(center_of_mass);

#ifdef VISUALIZE
	/* visual grid publication */
	sensor_msgs::ImagePtr grid_img =
	    cv_bridge::CvImage(std_msgs::Header(), "mono8", (*lprocessor).p_map).toImageMsg();
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
