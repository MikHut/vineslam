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
			Point<double> p_pos     = landmarks[j].image_pos[n];

			if (std::fabs(lc_pos[i].x - p_pos.x) < params.match_box &&
			    iter - last_iter <= 5 && iter - last_iter > 0) {
				Line<double> lp_line = computeLine(p_pos);
				Line<double> lc_line = computeLine(lc_pos[i]);

				matches.push_back(Match<double>(p_pos, lc_pos[i], lp_line, lc_line));
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

			index.push_back(landmarks.size() - 1);

			if (params.type == "pf") {
				bool side = (lc_pos[i].x < params.width / 2) ? 1 : 0;
				pf.push_back(PF(side, params));
			}
			else {
				Eigen::MatrixXd P0(2, 2);
				P0 << 2, 0, 0, 2;
				Line<double>  l = computeLine(lc_pos[i]);
				Point<double> X0(r_pos.x + 1, l.getY(r_pos.x + 1));

				KF kf_(P0, X0.eig(), params);
				kf.push_back(kf_);
			}
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
