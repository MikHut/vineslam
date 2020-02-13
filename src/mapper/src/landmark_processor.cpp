#include "../include/mapper/landmark_processor.hpp"

LandmarkProcessor::LandmarkProcessor(const Parameters& params) : params(params)
{
}

void LandmarkProcessor::updateDetections(
    const std::vector<Point<double>>& detections)
{
}

std::vector<Match<double>>
LandmarkProcessor::matchLandmarks(const std::vector<Point<double>>& l_det,
                                  const std::vector<Point<double>>& r_det)
{
	std::vector<Match<double>> matches;

	for (size_t i = 0; i < l_det.size(); i++) {
		Point<double> l_pos = l_det[i];
		for (size_t j = 0; j < r_det.size(); j++) {
			Point<double> r_pos = r_det[j];

			if (l_pos.x - r_pos.x < params.match_box && l_pos.x - r_pos.x > 0) {
				Line<double> l_line = computeLine(l_pos);
				Line<double> r_line = computeLine(r_pos);

				matches.push_back(Match<double>(l_pos, r_pos, l_line, r_line));
			}
		}
	}

	return matches;
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
