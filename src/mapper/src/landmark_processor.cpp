#include "../include/mapper/landmark_processor.hpp"

LandmarkProcessor::LandmarkProcessor(const Parameters& params) : params(params)
{
}

void LandmarkProcessor::updateDetections(
    const std::vector<Point<double>>& detections)
{
	lp_pos = lc_pos;
	lc_pos = detections;
}

void LandmarkProcessor::matchLandmarks(const int&           iter,
                                       const Point<double>& r_pos,
                                       std::vector<int>&    index)
{
	matches.clear();
	bool was_found;

	for (size_t i = 0; i < lc_pos.size(); i++) {
		was_found = false;
		for (size_t j = 0; j < landmarks.size(); j++) {
			int           n         = landmarks[j].image_pos.size() - 1;
			int           last_iter = landmarks[j].ptr[n];
			Point<double> p_pose    = landmarks[j].image_pos[n];

			if (std::fabs(lc_pos[i].x - p_pose.x) < params.match_box &&
			    iter - last_iter <= 5 && iter - last_iter > 0) {
				Line<double> lp_line = computeLine(p_pose);
				Line<double> lc_line = computeLine(lc_pos[i]);

				matches.push_back(Match<double>(p_pose, lc_pos[i], lp_line, lc_line));
				landmarks[j].image_pos.push_back(lc_pos[i]);
				landmarks[j].ptr.push_back(iter);

				index.push_back(j);

				was_found = true;
				break;
			}
		}

		if (was_found == false) {
			landmarks.push_back(Landmark<double>(landmarks.size(), lc_pos[i]));
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

cv::Mat LandmarkProcessor::projectToPlane(
    const cv::Mat& in, const std::vector<Line<double>>& trunks,
    const std::vector<Line<double>>& vine_lines, const Point<double>& robot_p)
{
	double z          = params.cam_height;
	double f          = params.cam_f;
	double cx         = params.ccd_h / 2;
	double cy         = params.ccd_w / 2;
	double phi        = params.phi;
	double pixel_size = params.ccd_w / params.width;
	int    grid_w     = params.grid_width;
	int    grid_h     = params.grid_length;
	int    grid_min   = params.grid_min;

	std::vector<Point<double>> vine_rpoints;
	std::vector<Point<double>> vine_lpoints;

	std::vector<std::vector<Point<double>>> trunks_array;
	trunks_array.resize(trunks.size());

	cv::Mat out(grid_w + grid_min, grid_h, in.type());
	out = 0;
	for (int i = grid_min; i < grid_min + grid_w; i++) {
		for (int j = -grid_h / 2; j < grid_h / 2; j++) {
			/* extrinsic transformation */
			double z_cam = i * cos(phi) - z * sin(phi);
			double y_cam = j;
			double x_cam = i * sin(phi) + z * cos(phi);

			/* intrinsic transformation */
			cv::Point2d uv_rect;
			uv_rect.x = (f * x_cam) / z_cam + cx;
			uv_rect.y = (f * y_cam) / z_cam + cy;

			int u = (int)(uv_rect.x / pixel_size);
			int v = (int)(uv_rect.y / pixel_size);

			if (u > 0 && u < in.rows && v > 0 && v < in.cols)
				out.at<cv::Vec3b>(i, j + grid_h / 2) = in.at<cv::Vec3b>(u, v);

			/* project trunks and vine lines onto the plane */
			for (size_t k = 0; k < trunks.size(); k++) {
				Line<double> l = trunks[k];
				if (l.dist(Point<double>(v, u)) < 0.5 && u > l.p1.y && u < l.p2.y) {
					out.at<cv::Vec3b>(i, j + grid_h / 2) = cv::Vec3b(255, 0, 0);
					trunks_array[k].push_back(Point<double>(i, j));
				}
			}
			if (vine_lines.size() >= 1) {
				Line<double> rline = vine_lines[0];
				double       th    = atan2(rline.a, rline.b);
				if (rline.dist(Point<double>(v, u)) < 0.5 && std::fabs(th) > 0.45 &&
				    std::fabs(th) < 2.8) {
					out.at<cv::Vec3b>(i, j + grid_h / 2) = cv::Vec3b(0, 255, 0);
					vine_rpoints.push_back(Point<double>(i, j));
				}
			}
			if (vine_lines.size() == 2) {
				Line<double> lline = vine_lines[1];
				double       th    = atan2(lline.a, lline.b);
				if (lline.dist(Point<double>(v, u)) < 0.5 && std::fabs(th) > 0.45 &&
				    std::fabs(th) < 2.8) {
					out.at<cv::Vec3b>(i, j + grid_h / 2) = cv::Vec3b(0, 0, 255);
					vine_lpoints.push_back(Point<double>(i, j));
				}
			}
		}
	}

	vine_lline = Line<double>(vine_lpoints);
	vine_rline = Line<double>(vine_rpoints);

	std::vector<Line<double>> trunk_lines;
	for (size_t i = 0; i < trunks_array.size(); i++)
		trunk_lines.push_back(Line<double>(trunks_array[i]));
	for (size_t i = 0; i < trunk_lines.size(); i++) {
		Point<double> pt;
		if (trunk_lines[i].p1.x > grid_w / 2)
			pt = trunk_lines[i].intercept(vine_rline);
		else
			pt = trunk_lines[i].intercept(vine_lline);

		cv::circle(out, cv::Point(pt.y + grid_h / 2, pt.x), 2,
		           cv::Scalar(255, 255, 255));
	}

	return out;
}
