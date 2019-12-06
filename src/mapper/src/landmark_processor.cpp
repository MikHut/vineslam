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

void LandmarkProcessor::matchLandmarks(const int& iter, std::vector<int>& index)
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

			index.push_back(landmarks.size());

			bool side = (lc_pos[i].x < params.width / 2) ? 1 : 0;
			pf.push_back(PF(side, params));
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

