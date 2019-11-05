#include "../include/mapper/mapper.hpp"

Estimator::Estimator(const Parameters&        params,
                     const LandmarkProcessor& lprocessor)
    : params(params), lprocessor(lprocessor)
{
}

void Estimator::init() {}

void Estimator::process(std::vector<Landmark<double>>&   landmarks,
                        const std::vector<Pose<double>>& robot_poses)
{
	std::vector<Pose<double>> filtered_poses;
	filterXYTheta(robot_poses, filtered_poses);
	predict(landmarks, filtered_poses);

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

void Estimator::predict(std::vector<Landmark<double>>&   landmarks,
                        const std::vector<Pose<double>>& robot_poses)
{
	int min_obsvs = 10;
	int comp      = params.filter_window;

	for (size_t i = 0; i < landmarks.size(); i++) {
		if (landmarks[i].image_pos.size() < min_obsvs)
			continue;


		Landmark<double> l = landmarks[i];
		Point<double>    avg(0, 0);

		/* int inc = l.image_pos.size() * 0.7; */
		int inc    = 50;
		int avg_it = 0;
		for (size_t j = inc + comp; j < l.image_pos.size(); j++) {
			Point<double> X_prev = l.image_pos[j - inc];
			Point<double> X_curr = l.image_pos[j];

			int          k       = l.ptr[j - comp];
			Pose<double> delta_p = robot_poses[k] - robot_poses[k - inc];

			Line<double> l_prev = lprocessor.computeLine(X_prev);
			Line<double> l_proj =
			    lprocessor.projectLine(X_curr, delta_p.pos, delta_p.theta);
			Point<double> X = l_prev.intercept(l_proj);

			X = robot_poses[k - inc].pos + X;
			if (X.x > 0 && X.x < 1000) {
				avg = (X + (avg * avg_it)) / (avg_it + 1);
				avg_it++;
			}
			all_res.push_back(X);
		}
		landmarks[i].world_pos = avg;
		std::cout << "RES: " << avg;
	}
}

void Estimator::drawMap(const std::vector<Pose<double>>& poses)
{
	map = cv::Mat(1000, 1000, CV_32F, cv::Scalar(0, 0, 0));
	for (size_t i = 0; i < poses.size(); i++) {
		cv::circle(map, cv::Point2f(poses[i].pos.x, poses[i].pos.y + 500), 2,
		           cv::Scalar(255, 0, 0), 2);
	}
	for (size_t i = 0; i < all_res.size(); i++) {
		cv::circle(map, cv::Point2f(all_res[i].x, all_res[i].y + 500), 2,
		           cv::Scalar(200, 0, 0), 2);
	}
	for (size_t i = 0; i < lprocessor.landmarks.size(); i++) {
		Point<double> pt = lprocessor.landmarks[i].world_pos;
		cv::circle(map, cv::Point2f(pt.x, pt.y + 500), 4, cv::Scalar(200, 200, 200),
		           4);
	}
}
