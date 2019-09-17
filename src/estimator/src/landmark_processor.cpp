#include "../include/landmark_processor.hpp"

#ifdef VISUALIZE
int color[6] = {220, 180, 140, 100, 60, 20};
#endif

LandmarkProcessor::LandmarkProcessor(const Parameters& params) : params(params)
{
	/* limits of landmark detection imposed by the camera FoV */
	x_start = tan(PI / 2 - params.v_fov / 2) * params.cam_height * 100;
	x_end   = TRUNK_SCOPE * 100;
}

void LandmarkProcessor::process(const std::vector<Point<double>>& poses,
                                std::vector<Particle<double>>&    particles)
{
#ifdef VISUALIZE
	grid = cv::Mat(params.resolution * 100, params.resolution * 100, CV_8UC1,
	               cv::Scalar(255, 255, 255));

	int p = 10575;

	std::cout << "Projecting particle number " << p << " | ";
	std::cout << "X = " << particles[p].pos.x << "  ,  ";
	std::cout << "Y = " << particles[p].pos.y << " | ";
	std::cout << "Theta = " << particles[p].theta << std::endl;
#endif

	updatePoses(poses);
	matchLandmarks();

	for (size_t i = 0; i < particles.size(); i++) {
		for (size_t j = 0; j < matches.size(); j++) {
			Line<double> pl = matches[j].p_line;
			Line<double> cl = matches[j].c_line;
			Line<double> tmp =
			    /* projectLine(matches[j], particles[i].pos, particles[i].theta); */
			    projectLine(matches[j], particles[i].pos, PI / 6);
			Point<double> X = cl.intercept(tmp);

			particles[i].weight += (X.x > x_start && X.x < x_end);

#ifdef VISUALIZE
			if (i == p) {
				plotGrid(matches[j].c_line, color[j]);
				plotGrid(tmp, color[j]);
			}
#endif
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
	    ((params.h_fov / 2) / params.width) * (landmark.x - params.width / 2);
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
	} while (pt.x > 0 && pt.x < x_end &&
	         std::fabs(pt.y) < params.resolution * 100 / 2);

	p2 = pt;

	return Line<double>(p1, p2);
}

Line<double> LandmarkProcessor::computeLine(const Point<double>& landmark,
                                            const double&        phi)
{
	double orientation =
	    ((params.h_fov / 2) / params.width) * (landmark.x - params.width / 2) +
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
	} while (pt.x > 0 && pt.x < x_end &&
	         std::fabs(pt.y) < params.resolution * 100 / 2);

	p2 = pt;

	return Line<double>(p1, p2);
}

Line<double> LandmarkProcessor::projectLine(const Match<double>& m,
                                            const Point<double>& delta_p,
                                            const double&        delta_th)
{
	/* First rotate the line */
	Line<double> l = computeLine(m.p_pos, delta_th);
	/* Then, translate the line */
	Point<double> p1(l.p1.x + delta_p.x * 100, l.p1.y + delta_p.y * 100);
	Point<double> p2(l.p2.x + delta_p.x * 100, l.p2.y + delta_p.y * 100);

	return Line<double>(p1, p2);
}

void LandmarkProcessor::plotGrid(const Line<double>& l, const int& color)
{
	Point<double> pt(l.p1.x, params.resolution * 100 / 2);
	while (pt.x >= 0 && pt.x < x_end && pt.y >= 0 &&
	       pt.y < params.resolution * 100) {
		pt.x += 1;
		pt.y = (l.c - l.a * pt.x) / l.b + params.resolution * 100 / 2;

		cv::circle(grid, cv::Point2f(pt.x, pt.y), 1,
		           cv::Scalar(color, color / 2, color / 3), 2);
		/* grid.at<uchar>(pt.y, pt.x) = color; */
	}
}
