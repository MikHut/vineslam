#include "../include/mapper/landmark_processor.hpp"

LandmarkProcessor::LandmarkProcessor(const Parameters& params) : params(params)
{
	/* limits of landmark detection imposed by the camera FoV */
	x_start = tan(PI / 2 - params.v_fov / 2) * params.cam_height;
	x_end   = TRUNK_SCOPE;
}

void LandmarkProcessor::updatePoses(const std::vector<Point<double>>& poses)
{
	lp_pose = lc_pose;
	lc_pose = poses;
}

void LandmarkProcessor::matchLandmarks(const Pose<double>& robot_pose)
{
	matches.clear();
	bool was_found;

	for (size_t i = 0; i < lc_pose.size(); i++) {
		was_found = false;
		for (size_t j = 0; j < landmarks.size(); j++) {
			int           n      = landmarks[j].image_pos.size() - 1;
			Point<double> p_pose = landmarks[j].image_pos[n];

			if (std::fabs(lc_pose[i].x - p_pose.x) < params.match_box) {
				Line<double> lp_line = computeLine(p_pose);
				Line<double> lc_line = computeLine(lc_pose[i]);

				matches.push_back(Match<double>(p_pose, lc_pose[i], lp_line, lc_line));
				landmarks[j].image_pos.push_back(lc_pose[i]);
				landmarks[j].r_pose.push_back(robot_pose);

				was_found = true;
				break;
			}
		}

		if (was_found == false) {
			landmarks.push_back(Landmark<double>(landmarks.size(), lc_pose[i]));
			landmarks[landmarks.size() - 1].r_pose.push_back(robot_pose);
		}
	}
}

Line<double> LandmarkProcessor::computeLine(const Point<double>& landmark)
{
	double orientation =
	    -(params.h_fov / params.width) * (params.width / 2 - landmark.x);

	Point<double> p1(0, 0);
	Point<double> p2(20 * cos(orientation), 20 * sin(orientation));

	return Line<double>(p1, p2);
}

Line<double> LandmarkProcessor::computeLine(const Point<double>& landmark,
                                            const double&        phi)
{
	double orientation =
	    -(params.h_fov / params.width) * (params.width / 2 - landmark.x);

	Point<double> p1(0, 0);
	Point<double> p2(20 * cos(orientation - phi), 20 * sin(orientation - phi));

	return Line<double>(p1, p2);
}

Line<double> LandmarkProcessor::projectLine(const Point<double>& pos,
                                            const Point<double>& delta_p,
                                            const double&        delta_th)
{
	/* First rotate the line */
	Line<double> l = computeLine(pos, delta_th);
	/* Then, translate the line */
	Point<double> p1 = l.p1 + delta_p;
	Point<double> p2 = l.p2 + delta_p;

	return Line<double>(p1, p2);
}

void LandmarkProcessor::plotGrid(const Line<double>& l, const int& color)
{
	double        x = (l.p1.x >= 0) ? l.p1.x : 0.0;
	Point<double> pt(x, params.resolution / 2);
	while (pt.x >= 0 && pt.x < x_end && pt.y >= 0 && pt.y < params.resolution) {
		pt.x += 1;
		pt.y = ((l.c - l.a * pt.x) / l.b) + (params.resolution / 2);

		cv::circle(p_map, cv::Point2f(pt.x, pt.y), 1,
		           cv::Scalar(color, color / 2, color / 3), 2);
	}
}

void LandmarkProcessor::plotPMap(Point<double>& p, const int& color)
{
	p.y = p.y + params.resolution / 2;
	if (p.x >= 0.0 && p.x < x_end && p.y >= 0 && p.y < params.resolution) {
		cv::circle(p_map, cv::Point2f(p.x, p.y), 7, cv::Scalar(color, 0, 0), 3);
	}
}
