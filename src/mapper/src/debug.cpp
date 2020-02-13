#include "../include/mapper/mapper_node.hpp"

void Mapper::showBBoxes(const sensor_msgs::ImageConstPtr& msg, cv::Mat& bboxes,
                        const std::vector<coral::DetectionCandidate>& res)
{
	std::vector<Point<double>> trunk_pos;
	for (auto result : res) {
		double xmin = result.corners.ymin * (*msg).width;
		double ymin = result.corners.xmin * (*msg).height;
		double xmax = result.corners.ymax * (*msg).width;
		double ymax = result.corners.xmax * (*msg).height;
		double xavg = (xmin + xmax) / 2;

		Point<double> tmp((xmin + xmax) / 2, (ymin + ymax) / 2);
		trunk_pos.push_back(tmp);

		if (result.score > (*params).min_score) {
			cv::rectangle(bboxes, cv::Point(xmin, ymin), cv::Point(xmax, ymax),
			              colors[result.label], 2);
			cv::line(bboxes, cv::Point(xavg, ymin), cv::Point(xavg, ymax),
			         cv::Scalar(0, 255, 0), 2);
		}
	}
}

void Mapper::showMatching(cv::Mat l_img, cv::Mat r_img)
{
	std::vector<cv::DMatch>   m;
	std::vector<cv::KeyPoint> key_point_right;
	std::vector<cv::KeyPoint> key_point_left;
	std::vector<cv::Point2f>  vec_right;
	std::vector<cv::Point2f>  vec_left;

	for (size_t i = 0; i < matches.size(); i++) {
		vec_right.push_back(cv::Point2f(matches[i].r_pos.x, matches[i].r_pos.y));
		vec_left.push_back(cv::Point2f(matches[i].l_pos.x, matches[i].l_pos.y));
	}
	cv::KeyPoint::convert(vec_right, key_point_right);
	cv::KeyPoint::convert(vec_left, key_point_left);

	for (size_t j = 0; j < vec_right.size(); j++)
		m.push_back(cv::DMatch(j, j, 0));

	cv::Mat img_matches;
	cv::drawMatches(l_img, key_point_left, r_img, key_point_right, m,
	                img_matches);

	sensor_msgs::ImagePtr out_img =
	    cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_matches).toImageMsg();
	matches_publisher.publish(out_img);
}
