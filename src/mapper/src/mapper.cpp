#include "../include/mapper/mapper.hpp"

Estimator::Estimator(const Parameters& params, LandmarkProcessor* lprocessor)
    : params(params), lprocessor(lprocessor), map_width(1000), map_heigth(1000)
{
}

void Estimator::process(const std::vector<Pose<double>>& robot_poses)
{
	std::vector<Pose<double>> filtered_poses;
	filterXYTheta(robot_poses, filtered_poses);
	predict(filtered_poses);
}

void Estimator::filterXYTheta(const std::vector<Pose<double>> robot_poses,
                              std::vector<Pose<double>>&      filtered_poses)
{
	int window     = params.filter_window;
	filtered_poses = robot_poses;

	for (size_t i = window - 1; i < robot_poses.size(); i++) {
		for (int j = 1; j < window; j++) {
			filtered_poses[i].pos.x += filtered_poses[i - j].pos.x;
			filtered_poses[i].pos.y += filtered_poses[i - j].pos.y;
			filtered_poses[i].theta += filtered_poses[i - j].theta;
		}
		filtered_poses[i].pos.x /= window;
		filtered_poses[i].pos.y /= window;
		filtered_poses[i].theta /= window;
	}
}

void Estimator::predict(const std::vector<Pose<double>>& robot_poses)
{
	std::vector<Landmark<double>> final_l;

	if (params.prediction == "histogram") {
		final_l = histogramPrediction(robot_poses);
		m_landmarks.clear();
		m_landmarks = final_l;
		drawHistogram(robot_poses);
	}
	else {
		final_l = averagePrediction(robot_poses);
		m_landmarks.clear();
		m_landmarks = final_l;
	}
	drawMap(robot_poses);
}

std::vector<Landmark<double>>
Estimator::averagePrediction(const std::vector<Pose<double>>& robot_poses)
{
	int max_std = params.max_stdev;
	int inc     = params.mapper_inc;
	int comp    = params.filter_window;

	std::vector<Landmark<double>> final_l;

	for (size_t i = 0; i < (*lprocessor).landmarks.size(); i++) {
		if ((*lprocessor).landmarks[i].image_pos.size() < inc)
			continue;

		std::vector<Point<double>> res;

		Landmark<double> l = (*lprocessor).landmarks[i];
		Point<double>    avg(0, 0);

		int avg_it = 0;
		for (size_t j = inc + comp; j < l.image_pos.size(); j++) {
			Point<double> X_prev = l.image_pos[j - inc];
			Point<double> X_curr = l.image_pos[j];

			int          k       = l.ptr[j - comp];
			Pose<double> delta_p = robot_poses[k] - robot_poses[k - inc];

			Line<double> l_prev = (*lprocessor).computeLine(X_prev);
			Line<double> l_proj =
			    (*lprocessor).projectLine(X_curr, delta_p.pos, delta_p.theta);
			Point<double> X = l_prev.intercept(l_proj);

			X   = robot_poses[k - inc].pos + X;
			avg = (X + (avg * avg_it)) / (avg_it + 1);
			avg_it++;

			res.push_back(X);
		}

		l.estimations = res;
		l.world_pos   = avg;
		l.standardDev();
		if (l.stdev.x < max_std)
			final_l.push_back(l);
	}

	return final_l;
}

std::vector<Landmark<double>>
Estimator::histogramPrediction(const std::vector<Pose<double>>& robot_poses)
{
	Grid<int> grid(map_width, map_heigth);

	int max_std = params.max_stdev / scaler;
	int inc     = params.mapper_inc;
	int comp    = params.filter_window;

	std::vector<Landmark<double>> final_l;

	for (size_t i = 0; i < (*lprocessor).landmarks.size(); i++) {
		if ((*lprocessor).landmarks[i].image_pos.size() < inc)
			continue;

		(*lprocessor).landmarks[i].cells = Grid<int>(map_width, map_heigth).cells;

		Landmark<double> l = (*lprocessor).landmarks[i];

		std::vector<Point<double>> res;
		for (size_t j = inc + comp; j < l.image_pos.size(); j++) {
			Point<double> X_prev = l.image_pos[j - inc];
			Point<double> X_curr = l.image_pos[j];

			int          k       = l.ptr[j - comp];
			Pose<double> delta_p = robot_poses[k] - robot_poses[k - inc];

			Line<double> l_prev = (*lprocessor).computeLine(X_prev);
			Line<double> l_proj =
			    (*lprocessor).projectLine(X_curr, delta_p.pos, delta_p.theta);
			Point<double> X = l_prev.intercept(l_proj);

			X = (robot_poses[k - inc].pos + X) / scaler;
			Point<int> index(std::round(X.x), std::round(X.y));

			res.push_back(X);

			int array_index = grid.arrayIndex(index);
			if (array_index >= 0) {
				grid.cells[array_index].score++;
				l.cells[array_index].score++;
			}
		}

		l.estimations = res;
		l.world_pos   = l.maxScore();
		l.standardDev();

		if (l.stdev.x < max_std)
			final_l.push_back(l);
	}

	(*lprocessor).grid.clean();
	(*lprocessor).grid = grid;

	return final_l;
}

void Estimator::filterMap(const std::vector<Pose<double>>& robot_poses)
{
	for (size_t i = 0; i < m_landmarks.size() - 3; i++) {
		std::vector<Landmark<double>> l;
		l.push_back(m_landmarks[i]);

		int k = 0, n = 1;
		while (k < 3 && (i + n) < m_landmarks.size()) {
			Landmark<double> l1 = m_landmarks[i + n];
			if (l1.world_pos.y * l[0].world_pos.y > 0) {
				l.push_back(m_landmarks[i + n]);
				k++;
			}
			n++;
		}

		/* Compute line that represents the landmark uncertainty */
		int           m   = l[0].estimations.size() / 2;
		Point<double> min = l[0].estimations[m - 2];
		Point<double> max = l[0].estimations[m + 2];
		Line<double>  unc_l(min, max);

		/* Compute average of a 4-size window of landmarks */
		double avg = 0;
		for (size_t j = 0; j < l.size(); j++)
			avg += l[j].world_pos.y;
		avg /= l.size();
		Line<double> avg_l(Point<double>(0, avg), Point<double>(500, avg));

		/* Update landmark position - filtered */
		Point<double> filt_pos = avg_l.intercept(unc_l);
		double        closest  = INF;
		for (size_t j = 0; j < l[0].estimations.size(); j++) {
			Point<double> pt   = l[0].estimations[j];
			double        dist = pt.euc_dist(filt_pos);
			if (dist < closest) {
				closest  = dist;
				filt_pos = pt;
			}
		}
		m_landmarks[i].world_pos = filt_pos;
	}
	drawMap(robot_poses);
}

void Estimator::drawMap(const std::vector<Pose<double>>& robot_poses)
{
	int width  = map_width;
	int height = map_heigth;

	map = cv::Mat(width, height, CV_8UC3, cv::Scalar(255, 255, 255));

	float m_scaler;
	if (params.prediction == "average")
		m_scaler = scaler;
	else
		m_scaler = 1;


	for (size_t i = 0; i < robot_poses.size(); i++) {
		Point<double> pt(robot_poses[i].pos.x / scaler + width / 10,
		                 robot_poses[i].pos.y / scaler + height / 2);

		if (pt.x > 0 && pt.y > 0 && pt.x < width && pt.y < height)
			cv::circle(map, cv::Point(pt.x, pt.y), 2, cv::Scalar(0, 0, 0), 2);
	}

	std::vector<Landmark<double>> l = m_landmarks;
	for (size_t i = 0; i < l.size(); i++) {
		for (size_t j = 0; j < l[i].estimations.size(); j++) {
			Point<double> pt(l[i].estimations[j].x / m_scaler + width / 10,
			                 l[i].estimations[j].y / m_scaler + height / 2);

			if (pt.x > 0 && pt.y > 0 && pt.x < width && pt.y < height)
				cv::circle(map, cv::Point(pt.x, pt.y), 2, colors[i], 2);
		}
	}

	for (size_t i = 0; i < l.size(); i++) {
		Point<double> pt =
		    l[i].world_pos / m_scaler + Point<double>(width / 10, height / 2);
		if (pt.x > 0 && pt.y > 0 && pt.x < width && pt.y < height)
			cv::circle(map, cv::Point(pt.x, pt.y), 4, cv::Scalar(0, 0, 255), 2);
	}
}

void Estimator::singleDraw(const std::vector<Pose<double>>& robot_poses,
                           const int&                       id)
{
	int width  = map_width;
	int height = map_heigth;

	single_map = cv::Mat(width, height, CV_8UC3, cv::Scalar(255, 255, 255));

	for (size_t i = 0; i < robot_poses.size(); i++) {
		Point<double> pt(robot_poses[i].pos.x / scaler + width / 10,
		                 robot_poses[i].pos.y / scaler + height / 2);

		if (pt.x > 0 && pt.y > 0 && pt.x < width && pt.y < height)
			cv::circle(single_map, cv::Point(pt.x, pt.y), 2, cv::Scalar(0, 0, 0), 2);
	}

	Landmark<double> l = m_landmarks[id];
	for (size_t j = 0; j < l.estimations.size(); j++) {
		Point<double> pt(l.estimations[j].x + width / 10,
		                 l.estimations[j].y + height / 2);

		if (pt.x > 0 && pt.y > 0 && pt.x < width && pt.y < height)
			cv::circle(single_map, cv::Point(pt.x, pt.y), 2, colors[id], 2);
	}

	float m_scaler;
	if (params.prediction == "average")
		m_scaler = scaler;
	else
		m_scaler = 1;

	Point<double> pt =
	    l.world_pos / m_scaler + Point<double>(width / 10, height / 2);
	if (pt.x > 0 && pt.y > 0 && pt.x < width && pt.y < height)
		cv::circle(single_map, cv::Point(pt.x, pt.y), 4, cv::Scalar(0, 0, 255), 2);
}

void Estimator::drawHistogram(const std::vector<Pose<double>>& robot_poses)
{
	Grid<int> grid = (*lprocessor).grid;

	int width  = map_width;
	int height = map_heigth;

	histogram = cv::Mat(width, height, CV_8UC3, cv::Scalar(255, 255, 255));

	for (size_t i = 0; i < robot_poses.size(); i++) {
		Point<double> pt(robot_poses[i].pos.x / scaler + width / 10,
		                 robot_poses[i].pos.y / scaler + height / 2);

		if (pt.x > 0 && pt.y > 0 && pt.x < width && pt.y < height)
			cv::circle(histogram, cv::Point(pt.x, pt.y), 2, cv::Scalar(0, 0, 0), 2);
	}

	std::vector<Cell<int>> cells = (*lprocessor).grid.cells;
	for (size_t i = 0; i < cells.size(); i++) {
		int        score = cells[i].score;
		Point<int> pt(cells[i].index.x + width / 10, cells[i].index.y + height / 2);

		if (score > 1 && pt.x > 0 && pt.y > 0 && pt.x < width && pt.y < height) {
			cv::circle(histogram, cv::Point(pt.x, pt.y), 2,
			           cv::Scalar(255 / score, 150 / score, 50 / score), 2);
		}
	}
}
