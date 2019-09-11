#include "../include/landmark_processor.hpp"

LandmarkProcessor::LandmarkProcessor(const Parameters& params) : params(params)
{
}

void LandmarkProcessor::updatePoses(const std::vector<Point<double>>& poses)
{
  lp_pose = lc_pose;
  lc_pose = poses;
}

cv::Mat LandmarkProcessor::plotGrid()
{
  /* initializes grid with cells of 1cm x 1cm dimension  */
  double  x_origin = params.resolution * 100 / 2;
  cv::Mat grid     = cv::Mat(params.resolution * 100, params.resolution * 100,
			     CV_8UC1, cv::Scalar(255, 255, 255));

  for (auto l : lc_pose) {
    double row = (l.y <= params.height / 2) ? l.y : l.y - params.height / 2;
    double y_origin =
	(params.cam_height * tan(pi / 2 - params.v_fov / 2) / 100 + row);

    double orientation =
	(l.x < params.width / 2) ? params.h_fov / 2 + pi / 2 : params.h_fov / 2;

    Point<double> pt(x_origin, y_origin);
    do {
      /* OpenCV convention (y,x) switched with the one adopted by ROS (x,y) */
      grid.at<uchar>(pt.y, pt.x) = 0;

      pt.x += sqrt(2) * cos(orientation);
      pt.y -= sqrt(2) * sin(orientation);

    } while (pt.x > 0 && pt.x < params.resolution * 100 && pt.y > 0 &&
	     pt.y < params.resolution * 100);
  }

  return grid;
}

std::vector<Point<double>>
LandmarkProcessor::computeLine(const Point<double>& landmark)
{
  double row = (landmark.y <= params.height / 2)
		   ? landmark.y
		   : landmark.y - params.height / 2;
  double x_origin = params.resolution * 100 / 2;
  double y_origin =
      params.cam_height * tan(pi / 2 - params.v_fov / 2) / 100 + row;
  double orientation = (landmark.x < params.width / 2)
			   ? params.h_fov / 2 + pi / 2
			   : params.h_fov / 2;

  Point<double>		     pt(x_origin, y_origin);
  std::vector<Point<double>> line;
  do {
    pt.x += sqrt(2) * cos(orientation);
    pt.y -= sqrt(2) * sin(orientation);

    line.push_back(pt);
  } while (pt.x > 0 && pt.x < params.resolution * 100 && pt.y > 0 &&
	   pt.y < params.resolution * 100);

  return line;
}

void LandmarkProcessor::matchLandmarks()
{
  matches.clear();
  for (size_t i = 0; i < lc_pose.size(); i++) {
    for (size_t j = 0; j < lp_pose.size(); j++)
      if (lc_pose[i].euc_dist(lp_pose[j]) < params.match_box)
	matches.push_back(Match<double>(lp_pose[j], lc_pose[i], params));
  }
}
