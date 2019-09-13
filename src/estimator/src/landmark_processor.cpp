#include "../include/landmark_processor.hpp"

LandmarkProcessor::LandmarkProcessor(const Parameters& params) : params(params) {}

void LandmarkProcessor::updatePoses(const std::vector<Point<double>>& poses)
{
  lp_pose = lc_pose;
  lc_pose = poses;
}

cv::Mat LandmarkProcessor::plotGrid()
{
  double  x_start  = tan(PI / 2 - params.v_fov / 2) * params.cam_height * 100;
  double  x_end    = TRUNK_SCOPE * 100;
  double  x_origin = 0;
  double  y_origin = 0;
  cv::Mat grid     = cv::Mat(params.resolution * 100, params.resolution * 100, CV_8UC1,
			     cv::Scalar(255, 255, 255));

  for (auto l : lc_pose) {
    double orientation = ((params.h_fov / 2) / params.width) * (l.x - params.width / 2);

    Point<double> pt(x_origin, y_origin + params.resolution * 100 / 2);
    do {
      /* OpenCV convention (y,x) switched with the one adopted by ROS (x,y) */
      if (pt.x >= x_start) grid.at<uchar>(pt.y, pt.x) = 0;

      pt.x += sqrt(2) * cos(orientation);
      pt.y += sqrt(2) * sin(orientation);

    } while (pt.x > 0 && pt.x < x_end && pt.y < params.resolution * 100);
  }

  return grid;
}

std::vector<Point<double>> LandmarkProcessor::computeLine(const Point<double>& landmark)
{
  double x_start  = tan(PI / 2 - params.v_fov / 2) * params.cam_height * 100;
  double x_end    = TRUNK_SCOPE * 100;
  double x_origin = 0;
  double y_origin = 0;
  double orientation =
      ((params.h_fov / 2) / params.width) * (landmark.x - params.width / 2);

  Point<double>		     pt(x_origin, y_origin);
  std::vector<Point<double>> line;
  do {
    /* Insert point on the line */
    if (pt.x >= x_start) line.push_back(pt);

    pt.x += sqrt(2) * cos(orientation);
    pt.y += sqrt(2) * sin(orientation);

  } while (pt.x > 0 && pt.x < x_end && std::fabs(pt.y) < params.resolution * 100 / 2);

  return line;
}

std::vector<Point<double>> LandmarkProcessor::computeLine(const Point<double>& landmark,
							  const double& orientation)
{
  double x_start  = tan(PI / 2 - params.v_fov / 2) * params.cam_height * 100;
  double x_end    = TRUNK_SCOPE * 100;
  double x_origin = 0;
  double y_origin = 0;

  Point<double>		     pt(x_origin, y_origin);
  std::vector<Point<double>> line;
  do {
    /* Insert point on the line */
    if (pt.x >= x_start) line.push_back(pt);

    pt.x += sqrt(2) * cos(orientation);
    pt.y += sqrt(2) * sin(orientation);

  } while (pt.x > 0 && pt.x < x_end && std::fabs(pt.y) < params.resolution * 100 / 2);

  return line;
}

void LandmarkProcessor::matchLandmarks()
{
  matches.clear();
  for (size_t i = 0; i < lc_pose.size(); i++) {
    for (size_t j = 0; j < lp_pose.size(); j++)
      if (lc_pose[i].euc_dist(lp_pose[j]) < params.match_box)
	      matches.push_back(Match<double>(lp_pose[j], lc_pose[i], computeLine(lp_pose[j]),
					computeLine(lc_pose[i]), params));
  }
}

std::vector<Point<double>>
LandmarkProcessor::projectLine(const std::vector<Point<double>>& l,
			       const int&			 particle_id)
{
  double x_origin = particles[particle_id].x;
  double y_origin = particles[particle_id].y;
  double orientation =
      ((params.h_fov / 2) / params.width) * (landmark.x - params.width / 2) +
      particles[particle_id].theta;

  return computeLine(Point<double>(x_origin, y_origin), orientation);
}
