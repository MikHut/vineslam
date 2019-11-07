#include "../include/mapper/mapper.hpp"

Estimator::Estimator(const Parameters& params, LandmarkProcessor* lprocessor)
    : params(params), lprocessor(lprocessor)
{
}

void Estimator::init() {}

void Estimator::process(const std::vector<Pose<double>>& robot_poses)
{
	std::vector<Pose<double>>  filtered_poses;
	std::vector<Point<double>> trunk_poses;
	filterXYTheta(robot_poses, filtered_poses);
	predict(filtered_poses, trunk_poses);

	drawMap(filtered_poses, trunk_poses);
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

void Estimator::predict(const std::vector<Pose<double>>& robot_poses,
                        std::vector<Point<double>>&      trunk_poses)
{
	int max_std   = 1000;
	int min_obsvs = 50;
	int comp      = params.filter_window;

	for (size_t i = 0; i < (*lprocessor).landmarks.size(); i++) {
		if ((*lprocessor).landmarks[i].image_pos.size() < min_obsvs)
			continue;

		Landmark<double> l = (*lprocessor).landmarks[i];
		Point<double>    avg(0, 0);

		/* int inc = l.image_pos.size() * 0.7; */
		int inc    = 50;
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

			all_res.push_back(X);
		}
		trunk_poses.push_back(avg);
	}
}

void Estimator::drawMap(const std::vector<Pose<double>>&  robot_poses,
                        const std::vector<Point<double>>& trunk_poses)
{
	map = cv::Mat(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
	for (size_t i = 0; i < robot_poses.size(); i++) {
		cv::circle(map,
		           cv::Point(robot_poses[i].pos.x / 10 + 100,
		                     robot_poses[i].pos.y / 10 + 250),
		           2, cv::Scalar(0, 0, 0), 2);
	}
	for (size_t i = 0; i < all_res.size(); i++) {
		cv::circle(map, cv::Point(all_res[i].x / 10 + 100, all_res[i].y / 10 + 250),
		           2, cv::Scalar(255, 0, 0), 2);
	}
	for (size_t i = 0; i < trunk_poses.size(); i++) {
		Point<double> pt = trunk_poses[i];
		cv::circle(map, cv::Point(pt.x / 10 + 100, pt.y / 10 + 250), 4,
		           cv::Scalar(0, 0, 255), 2);
	}
}
