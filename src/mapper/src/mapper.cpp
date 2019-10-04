#include "../include/mapper/mapper.hpp"

Estimator::Estimator(const Parameters& params) : params(params)
{
	prev_pose = Pose<double>(Point<double>(0, 0), 0);
}

void Estimator::init() {}

void Estimator::process(const std::vector<Landmark<double>>& landmarks,
                        const std::vector<Pose<double>>&     robot_poses)
{
	std::vector<Pose<double>> filtered_poses;
	filterXYTheta(robot_poses, filtered_poses);
}

void Estimator::filterXYTheta(const std::vector<Pose<double>> robot_poses,
                              std::vector<Pose<double>>&      filtered_poses)
{
	int window = params.filter_window;
	filtered_poses.resize(robot_poses.size());

	for (int i = 0; i < window; i++)
		filtered_poses[i] = robot_poses[i];

	for (size_t i = window; i < robot_poses.size(); i++) {
		for (int j = 0; j < window; j++) {
			filtered_poses[i].pos.x += robot_poses[i - j].pos.x;
			filtered_poses[i].pos.y += robot_poses[i - j].pos.y;
			filtered_poses[i].theta += robot_poses[i - j].theta;
		}
		filtered_poses[i].pos.x /= window;
		filtered_poses[i].pos.y /= window;
		filtered_poses[i].theta /= window;
	}

  drawMap(filtered_poses);
}

void Estimator::drawMap(const std::vector<Pose<double>>& poses)
{
	map = cv::Mat(1000, 1000, CV_32S, cv::Scalar(255, 255, 255));
  std::cout << poses.size() << std::endl;
	for (size_t i = 0; i < poses.size(); i++) {
    std::cout << poses[i];
		cv::circle(map, cv::Point2f(poses[i].pos.x, poses[i].pos.y + 500), 3,
		           cv::Scalar(0, 0, 0), 3);
  }
}

void Estimator::predict(const std::vector<Point<double>>& landm_pos,
                        const Pose<double>&               delta_pose)
{
}
