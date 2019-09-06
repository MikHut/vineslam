#include "../include/landmark_processor.hpp"

LandmarkProcessor::LandmarkProcessor(int w, int h, int res)
    : width(w), height(h), resolution(res)
{
}

void LandmarkProcessor::updatePoses(const std::vector<Point<int>> poses)
{
  previous_landmark_poses = landmark_poses;
  landmark_poses = poses;
}

cv::Mat LandmarkProcessor::buildGrid()
{
  /* initializes grid with cells of 1cm x 1cm dimension  */
  int     x_origin = resolution * 100 / 2;
  int     y_origin = resolution * 100;
  cv::Mat grid     = cv::Mat(resolution * 100, resolution * 100, CV_8UC1,
			     cv::Scalar(255, 255, 255));

  for (auto l : landmark_poses) {
    double orientation = atan2(height - l.y, l.x - width / 2);

    std::vector<Point<double>> lines;
    Point<double>	      pt(x_origin, y_origin);
    do {
      /* OpenCV convention (y,x) switched with the one adopted by ROS (x,y) */
      grid.at<uchar>(pt.y, pt.x) = 0;

      pt.x += sqrt(2) * cos(orientation);
      pt.y -= sqrt(2) * sin(orientation);

    } while (pt.x > 0 && pt.x < resolution * 100 && pt.y > 0 &&
	     pt.y < resolution * 100);
  }

  return grid;
}

void LandmarkProcessor::matchLandmarks() {
}
