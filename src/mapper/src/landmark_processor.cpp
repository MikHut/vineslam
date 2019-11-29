#include "../include/mapper/landmark_processor.hpp"

LandmarkProcessor::LandmarkProcessor(const Parameters& params) : params(params)
{
}

void LandmarkProcessor::updatePoses(const std::vector<Point<double>>& poses)
{
	lp_pose = lc_pose;
	lc_pose = poses;
}

void LandmarkProcessor::matchLandmarks(const int& iter)
{
	matches.clear();
	bool was_found;

	for (size_t i = 0; i < lc_pose.size(); i++) {
		was_found = false;
		for (size_t j = 0; j < landmarks.size(); j++) {
			int           n         = landmarks[j].image_pos.size() - 1;
			int           last_iter = landmarks[j].ptr[n];
			Point<double> p_pose    = landmarks[j].image_pos[n];

			if (std::fabs(lc_pose[i].x - p_pose.x) < params.match_box &&
			    iter - last_iter <= 5 && iter - last_iter > 0) {
				Line<double> lp_line = computeLine(p_pose);
				Line<double> lc_line = computeLine(lc_pose[i]);

				matches.push_back(Match<double>(p_pose, lc_pose[i], lp_line, lc_line));
				landmarks[j].image_pos.push_back(lc_pose[i]);
				landmarks[j].ptr.push_back(iter);

				was_found = true;
				break;
			}
		}

		if (was_found == false) {
			landmarks.push_back(Landmark<double>(landmarks.size(), lc_pose[i]));
			landmarks[landmarks.size() - 1].ptr.push_back(iter);
		}
	}
}

Line<double> LandmarkProcessor::computeLine(const Point<double>& landmark)
{
	double orientation =
	    -(params.h_fov / params.width) * (params.width / 2 - landmark.x);

	Point<double> p1(0, 0);
	Point<double> p2(500 * cos(orientation), 500 * sin(orientation));

	return Line<double>(p1, p2);
}

Line<double> LandmarkProcessor::computeLine(const Point<double>& landmark,
                                            const double&        phi)
{
	double orientation =
	    -(params.h_fov / params.width) * (params.width / 2 - landmark.x);

	Point<double> p1(0, 0);
	Point<double> p2(500 * cos(orientation - phi), 500 * sin(orientation - phi));

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

cv::Mat LandmarkProcessor::projectToGround(const cv::Mat&                    in,
                                           const std::vector<Point<double>>& pt)
{
	/* camera info */
	float fx = params.cam_hf;
	float fy = params.cam_vf;
	float cx = params.width / 2;
	float cy = params.height / 2;

	float tz = params.cam_height / 100;

	cv::Mat out(400, 400, in.type());
	out = 0;
	for (int i = 0; i < out.cols; i++) {
		for (int j = 0; j < out.rows; j++) {
			float y = -((float)j - out.cols / 2) / 100;
			float z = (float)i / 100;
			float x = tz;

			cv::Point2d uv_rect;
			uv_rect.x = (fx * x) / z + cx;
			uv_rect.y = (fy * y) / z + cy;

			int u = uv_rect.x;
			int v = uv_rect.y;
			if (u > 0 && u < in.rows && v > 0 && v < in.cols) {
				out.at<cv::Vec3b>(i, j) = in.at<cv::Vec3b>(u, v);
			}
		}
	}

	/* project line on the ground */
	std::cout << pt.size() << std::endl;
	for (size_t i = 0; i < pt.size(); i++) {
		double th = (-params.h_fov / params.width) * (params.width / 2 - pt[i].x);
		Point<double> p(200, 0);
		while (p.x >= 0 && p.x < 400 && p.y >= 0 && p.y < 400) {
			cv::circle(out, cv::Point2d(p.x, p.y), 1, cv::Scalar(255, 255, 255), 1);

			p.x += sqrt(2) * cos(th);
			p.y += sqrt(2) * sin(th);
		}
	}

	return out;
}
