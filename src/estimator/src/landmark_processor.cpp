#include "../include/landmark_processor.hpp"

LandmarkProcessor::LandmarkProcessor(const Parameters& params) : params(params)
{
  /* limits of landmark detection imposed by the camera FoV */
  x_start = tan(PI / 2 - params.v_fov / 2) * params.cam_height * 100;
  x_end   = TRUNK_SCOPE * 100;
}

void LandmarkProcessor::process(const std::vector<Point<double>>& poses,
				std::vector<Particle<double>>&    particles)
{
  updatePoses(poses);
  matchLandmarks();

  for (size_t i = 0; i < particles.size(); i++)
  {
    for (size_t j = 0; j < matches.size(); j++)
    {
      Line<double>  pl  = matches[j].p_line;
      Line<double>  cl  = matches[j].c_line;
      Line<double>  tmp = projectLine(pl, particles[i].pos);
      Point<double> X   = cl.intercept(tmp);

      particles[i].weight += (X.x > x_start && X.x < x_end);
    }
  }
}

void LandmarkProcessor::updatePoses(const std::vector<Point<double>>& poses)
{
  lp_pose = lc_pose;
  lc_pose = poses;
}

void LandmarkProcessor::matchLandmarks()
{
  matches.clear();
  for (size_t i = 0; i < lc_pose.size(); i++)
  {
    for (size_t j = 0; j < lp_pose.size(); j++)
    {
      if (lc_pose[i].euc_dist(lp_pose[j]) < params.match_box)
      {
	Line<double> lp_line = computeLine(lp_pose[j]);
	Line<double> lc_line = computeLine(lc_pose[i]);
	matches.push_back(Match<double>(lp_pose[j], lc_pose[i], lp_line, lc_line));
      }
    }
  }
}

Line<double> LandmarkProcessor::computeLine(const Point<double>& landmark)
{
  double orientation =
      ((params.h_fov / 2) / params.width) * (landmark.x - params.width / 2);
  bool p1_flag = false;

  Point<double> pt(0, 0);
  Point<double> p1;
  Point<double> p2;
  do
  {

    if (pt.x >= x_start && p1_flag == false)
    {
      p1      = pt;
      p1_flag = true;
    }

    pt.x += sqrt(2) * cos(orientation);
    pt.y += sqrt(2) * sin(orientation);

  } while (pt.x > 0 && pt.x < x_end && std::fabs(pt.y) < params.resolution * 100 / 2);

  p2 = pt;

  return Line<double>(p1, p2);
}

Line<double> LandmarkProcessor::projectLine(const Line<double>&  l,
					    const Point<double>& delta)
{
  Point<double> p1(l.p1.x - delta.x, l.p1.y - delta.y);
  Point<double> p2(l.p2.x - delta.x, l.p2.y - delta.y);

  return Line<double>(p1, p2);
}

cv::Mat LandmarkProcessor::plotGrid()
{
  double  x_start  = tan(PI / 2 - params.v_fov / 2) * params.cam_height * 100;
  double  x_end    = TRUNK_SCOPE * 100;
  double  x_origin = 0;
  double  y_origin = 0;
  cv::Mat grid     = cv::Mat(params.resolution * 100, params.resolution * 100, CV_8UC1,
			     cv::Scalar(255, 255, 255));

  for (auto l : lc_pose)
  {
    double orientation = ((params.h_fov / 2) / params.width) * (l.x - params.width / 2);

    Point<double> pt(x_origin, y_origin + params.resolution * 100 / 2);
    do
    {
      /* OpenCV convention (y,x) switched with the one adopted by ROS (x,y) */
      if (pt.x >= x_start) grid.at<uchar>(pt.y, pt.x) = 0;

      pt.x += sqrt(2) * cos(orientation);
      pt.y += sqrt(2) * sin(orientation);

    } while (pt.x > 0 && pt.x < x_end && pt.y < params.resolution * 100);
  }

  return grid;
}
