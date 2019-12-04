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
	else if (params.prediction == "average") {
		final_l = averagePrediction(robot_poses);
		m_landmarks.clear();
		m_landmarks = final_l;
	}
	else {
		final_l = kfPrediction(robot_poses);
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
			int           k = l.ptr[j - comp];
			Point<double> X =
			    processObsv(l, j, robot_poses[k] - robot_poses[k - inc]);
			X = robot_poses[k - inc].pos + X;

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

	int max_std = params.max_stdev * scaler;
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
			int           k = l.ptr[j - comp];
			Point<double> X =
			    processObsv(l, j, robot_poses[k] - robot_poses[k - inc]);
			X = (robot_poses[k - inc].pos + X) * scaler;

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

std::vector<Landmark<double>>
Estimator::kfPrediction(const std::vector<Pose<double>>& robot_poses)
{
	int max_std = params.max_stdev;
	int inc     = params.mapper_inc;
	int comp    = params.filter_window;

	std::vector<Landmark<double>> final_l;

	std::vector<Point<double>> map = initMap(100, 4);

	for (size_t i = 0; i < (*lprocessor).landmarks.size(); i++) {
		if ((*lprocessor).landmarks[i].image_pos.size() < inc)
			continue;

		Landmark<double> l = (*lprocessor).landmarks[i];

		/* find map landmark correspondence */
		Point<double> tmp = correspond(robot_poses[l.ptr[0]].pos, l.image_pos[0].x, map);

		/* initialize kalman filter */
		VectorXd TH = tmp.eig();
		MatrixXd P(2, 2);
		P << 0.4, 0, 0, 0.2;
		KF kf(TH, P, params);

		Point<double>              avg(0, 0);
		std::vector<Point<double>> res;

		int avg_it = 0;
		for (size_t j = inc + comp; j < l.image_pos.size(); j++) {
			/* extraction of the observation */
			int           k = l.ptr[j - comp];
			Point<double> X =
			    processObsv(l, j, robot_poses[k] - robot_poses[k - inc]);

			VectorXd z(2, 1);
			z << sqrt((X.x * X.x) + (X.y * X.y)), atan2(X.y, X.x);

			/* kalman filter */
			Point<double> s = robot_poses[k - inc].pos;
			kf.process(s.eig(), z);
			TH = kf.getState();

			/* estimation storage */
			res.push_back(Point<double>(TH[0], TH[1]));

			/* calculate average from all the estimations */
			avg = (Point<double>(TH[0], TH[1]) + (avg * avg_it)) / (avg_it + 1);
			avg_it++;
		}

		l.estimations = res;
		l.world_pos   = avg;
		l.standardDev();

		final_l.push_back(l);
	}

	return final_l;
}

Point<double> Estimator::processObsv(const Landmark<double>& l, const int& it,
                                     const Pose<double>& delta_p)
{
	int comp = params.filter_window;
	int inc  = params.mapper_inc;

	Point<double> X_prev = l.image_pos[it - inc];
	Point<double> X_curr = l.image_pos[it];

	Line<double> l_prev = (*lprocessor).computeLine(X_prev);
	Line<double> l_proj =
	    (*lprocessor).projectLine(X_curr, delta_p.pos, delta_p.theta);
	Point<double> X = l_prev.intercept(l_proj);

	return X;
}

std::vector<Point<double>> Estimator::initMap(const int& N_x, const int& N_y)
{
	double x_inc = params.vine_x;
	double y_inc = params.vine_y;

	std::vector<double> c_index;

	double y = (1 - N_y) * y_inc + y_inc / 2;
	for (int i = 0; i < (N_y - 1) * 2; i++) {
		c_index.push_back(y);
		y += y_inc;
	}

	std::vector<Point<double>> map;
	for (size_t i = 0; i < c_index.size(); i++) {
		for (int j = 0; j < N_x; j++) {
			Point<double> pt(-N_x / 2 + j * x_inc, c_index[i]);
			map.push_back(pt);
		}
	}

	return map;
}

Point<double> Estimator::correspond(const Point<double>& r, const int& col,
                                    const std::vector<Point<double>>& map)
{
	double th    = columnToTheta(col);
	double y_len = params.vine_y;
	double min   = INF;

	Point<double> l(0, 0);
	Line<double>  line(r, Point<double>(10 * cos(th), 10 * sin(th)));
	for (size_t i = 0; i < map.size(); i++) {
		if (std::fabs(r.y - map[i].y) < y_len && (map[i].x - r.x) > 0) {
			double d = line.dist(map[i]);
			if (d < min) {
				min = d;
				l   = map[i];
			}
		}
	}

	return l;
}
