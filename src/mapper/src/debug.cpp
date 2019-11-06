#include "../include/mapper/mapper_node.hpp"

void Mapper::showMatching(const sensor_msgs::ImageConstPtr& msg)
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
    vecRight.push_back(
	cv::Point2f((*lprocessor).matches[i].c_pos.x, (*lprocessor).matches[i].c_pos.y));
    vecLeft.push_back(
	cv::Point2f((*lprocessor).matches[i].p_pos.x, (*lprocessor).matches[i].p_pos.y));
  }
  cv::KeyPoint::convert(vecRight, keyPointRight);
  cv::KeyPoint::convert(vecLeft, keyPointLeft);

  for (size_t j = 0; j < vecRight.size(); j++) m.push_back(cv::DMatch(j, j, 0));

  cv::Mat img_matches;
  cv::drawMatches(p_image, keyPointLeft, c_image, keyPointRight, m,
		  img_matches);

  sensor_msgs::ImagePtr img =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_matches).toImageMsg();
  matches_publisher.publish(img);
}
