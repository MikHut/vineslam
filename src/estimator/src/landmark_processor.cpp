#include "../include/landmark_processor.hpp"

LandmarkProcessor::LandmarkProcessor(const Parameters& params) : params(params)
{
	/* limits of landmark detection imposed by the camera FoV */
	x_start = tan(PI / 2 - params.v_fov / 2) * params.cam_height;
	/* x_start = 0; */
	x_end   = TRUNK_SCOPE;
}

void LandmarkProcessor::updatePoses(const std::vector<Point<double>>& poses)
{
	lp_pose = lc_pose;
	lc_pose = poses;
}

void LandmarkProcessor::matchLandmarks()
{
	matches.clear();
	for (size_t i = 0; i < lc_pose.size(); i++) {
		for (size_t j = 0; j < lp_pose.size(); j++) {
			if (lc_pose[i].euc_dist(lp_pose[j]) < params.match_box) {
				Line<double> lp_line = computeLine(lp_pose[j]);
				Line<double> lc_line = computeLine(lc_pose[i]);
				matches.push_back(
				    Match<double>(lp_pose[j], lc_pose[i], lp_line, lc_line));
			}
		}
	}
}

Line<double> LandmarkProcessor::computeLine(const Point<double>& landmark)
{
	double orientation =
	    (params.h_fov / params.width) * (landmark.x - params.width / 2);
	    /* (params.h_fov / params.width) * (params.width / 2  - landmark.x); */
	bool p1_flag = false;

	Point<double> pt(0, 0);
	Point<double> p1;
	Point<double> p2;
	do {

		if (pt.x >= x_start && p1_flag == false) {
			p1      = pt;
			p1_flag = true;
		}

		pt.x += sqrt(2) * cos(orientation);
		pt.y += sqrt(2) * sin(orientation);
	} while (pt.x > 0 && pt.x < x_end && std::fabs(pt.y) < params.resolution / 2);

	p2 = pt;

	return Line<double>(p1, p2);
}

Line<double> LandmarkProcessor::computeLine(const Point<double>& landmark,
                                            const double&        phi)
{
	double orientation =
	    (params.h_fov / params.width) * (landmark.x - params.width / 2) +
	    /* (params.h_fov / params.width) * (params.width / 2  - landmark.x); */
	    phi;

	bool p1_flag = false;

	Point<double> pt(0, 0);
	Point<double> p1;
	Point<double> p2;
	do {

		if (pt.x >= x_start && p1_flag == false) {
			p1      = pt;
			p1_flag = true;
		}

		pt.x += sqrt(2) * cos(orientation);
		pt.y += sqrt(2) * sin(orientation);
	} while (pt.x > 0 && pt.x < x_end && std::fabs(pt.y) < params.resolution / 2);

	p2 = pt;

	return Line<double>(p1, p2);
}

Line<double> LandmarkProcessor::projectLine(const Match<double>& m,
                                            const Point<double>& delta_p,
                                            const double&        delta_th)
{
	/* First rotate the line */
	Line<double> l = computeLine(m.c_pos, delta_th);
	/* Then, translate the line */
	Point<double> p1(l.p1.x + delta_p.x, l.p1.y + delta_p.y);

	return Line<double>(p1, l.p2);
}

void LandmarkProcessor::plotGrid(const Line<double>& l, const int& color)
{
	double        x = (l.p1.x >= 0) ? l.p1.x : 0.0;
	Point<double> pt(x, params.resolution / 2);
	while (pt.x >= 0 && pt.x < x_end && pt.y >= 0 && pt.y < params.resolution) {
		pt.x += 1;
		pt.y = (params.resolution / 2) - ((l.c - l.a * pt.x) / l.b);

		cv::circle(p_map, cv::Point2f(pt.x, pt.y), 1,
		           cv::Scalar(color, color / 2, color / 3), 2);
	}
}

void LandmarkProcessor::plotPMap(Point<double>& p, const int& color)
{
	p.y = params.resolution / 2 - p.y;
	if (p.x >= 0.0 && p.x < x_end && p.y >= 0 && p.y < params.resolution) {
		cv::circle(p_map, cv::Point2f(p.x, p.y), 7, cv::Scalar(color, 0, 0), 3);
	}
}
