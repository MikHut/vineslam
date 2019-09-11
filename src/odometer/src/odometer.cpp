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

  processor = new LandmarkProcessor(*params);
}

void Odometer::boxListener(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
  /* center of mass calculation */
  std::vector<Point<double>> center_of_mass;
  for (auto i : (*msg).bounding_boxes) {
    Point<double> tmp((i.xmin + i.xmax) / 2, (i.ymin + i.ymax) / 2);
    center_of_mass.push_back(tmp);
  }

  /* landmark matching procedure */
  (*processor).updatePoses(center_of_mass);
  (*processor).matchLandmarks();

#ifdef VISUALIZE
  /* visual grid publication */
  cv::Mat grid = (*processor).plotGrid();
  cv::Mat concat;
  cv::hconcat(grid, last_grid, concat);
  sensor_msgs::ImagePtr img =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", concat).toImageMsg();
  grid_pub.publish(img);

  last_grid = grid;
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

  for (size_t i = 0; i < (*processor).matches.size(); i++) {
    vecRight.push_back(
	cv::Point2f((*processor).matches[i].c.x, (*processor).matches[i].c.y));
    vecLeft.push_back(
	cv::Point2f((*processor).matches[i].p.x, (*processor).matches[i].p.y));
  }
  cv::KeyPoint::convert(vecRight, keyPointRight);
  cv::KeyPoint::convert(vecLeft, keyPointLeft);

  for (size_t j = 0; j < vecRight.size(); j++) m.push_back(cv::DMatch(j, j, 0));

  cv::Mat img_matches;
  cv::drawMatches(p_image, keyPointLeft, c_image, keyPointRight, m,
		  img_matches);

  sensor_msgs::ImagePtr img =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_matches).toImageMsg();
  matches_pub.publish(img);
}
