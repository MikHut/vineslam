#include "../include/mapper/mapper.hpp"

Estimator::Estimator(const Parameters& params, LandmarkProcessor* lprocessor)
    : params(params), lprocessor(lprocessor)
{
}

void Estimator::init() {}

void Estimator::process(const std::vector<Pose<double>>& robot_poses)
{
	std::vector<Pose<double>>  filtered_poses;
	filterXYTheta(robot_poses, filtered_poses);
	predict(filtered_poses);

	drawMap(filtered_poses);
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
	int max_std = 1000;
	int inc     = 100;
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
    if(l.stdev.x < 1000)
		  final_l.push_back(l);
	}

	(*lprocessor).landmarks.clear();
	(*lprocessor).landmarks = final_l;
}

void Estimator::drawMap(const std::vector<Pose<double>>&  robot_poses)
{
	map = cv::Mat(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
	for (size_t i = 0; i < robot_poses.size(); i++) {
		cv::circle(map,
		           cv::Point(robot_poses[i].pos.x / 10 + 100,
		                     robot_poses[i].pos.y / 10 + 250),
		           2, cv::Scalar(0, 0, 0), 2);
	}
	std::vector<Landmark<double>> l = (*lprocessor).landmarks;
	for (size_t i = 0; i < l.size(); i++) {
		for (size_t j = 0; j < l[i].estimations.size(); j++) {
			cv::circle(map,
			           cv::Point(l[i].estimations[j].x / 10 + 100,
			                     l[i].estimations[j].y / 10 + 250),
			           2, colors[i], 2);
		}
	}
	for (size_t i = 0; i < l.size(); i++) {
		Point<double> pt = l[i].world_pos;
		cv::circle(map, cv::Point(pt.x / 10 + 100, pt.y / 10 + 250), 4,
		           cv::Scalar(0, 0, 255), 2);
	}
}

void Estimator::singleDraw(const std::vector<Pose<double>>& robot_poses,
                           const int&                       id)
{
	single_map = cv::Mat(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
	for (size_t i = 0; i < robot_poses.size(); i++) {
		cv::circle(single_map,
		           cv::Point(robot_poses[i].pos.x / 10 + 100,
		                     robot_poses[i].pos.y / 10 + 250),
		           2, cv::Scalar(0, 0, 0), 2);
	}
	Landmark<double> l = (*lprocessor).landmarks[id];
	for (size_t j = 0; j < l.estimations.size(); j++) {
		cv::circle(
		    single_map,
		    cv::Point(l.estimations[j].x / 10 + 100, l.estimations[j].y / 10 + 250),
		    2, colors[id], 2);
	}
	Point<double> pt = l.world_pos;
	cv::circle(single_map, cv::Point(pt.x / 10 + 100, pt.y / 10 + 250), 4,
	           cv::Scalar(0, 0, 255), 2);
}
