#include "../include/mapper/mapper.hpp"

void Estimator::drawMap(const std::vector<Pose<double>>& robot_poses)
{
	int width  = map_width;
	int height = map_heigth;

	map = cv::Mat(width, height, CV_8UC3, cv::Scalar(255, 255, 255));

	float m_scaler;
	if (params.prediction == "histogram")
		m_scaler = 1;
	else
		m_scaler = scaler;


	for (size_t i = 0; i < robot_poses.size(); i++) {
		Point<double> pt(robot_poses[i].pos.x * scaler + width / 10,
		                 robot_poses[i].pos.y * scaler + height / 2);

		if (pt.x > 0 && pt.y > 0 && pt.x < width && pt.y < height)
			cv::circle(map, cv::Point(pt.x, pt.y), 2, cv::Scalar(0, 0, 0), 2);
	}

	std::vector<Landmark<double>> l = m_landmarks;
	for (size_t i = 0; i < l.size(); i++) {
		for (size_t j = 0; j < l[i].estimations.size(); j++) {
			Point<double> pt(l[i].estimations[j].x * m_scaler + width / 10,
			                 l[i].estimations[j].y * m_scaler + height / 2);

			if (pt.x > 0 && pt.y > 0 && pt.x < width && pt.y < height)
				cv::circle(map, cv::Point(pt.x, pt.y), 2, colors[i], 2);
		}
	}

	for (size_t i = 0; i < l.size(); i++) {
		Point<double> pt =
		    l[i].world_pos * m_scaler + Point<double>(width / 10, height / 2);
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
		Point<double> pt(robot_poses[i].pos.x * scaler + width / 10,
		                 robot_poses[i].pos.y * scaler + height / 2);

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
	    l.world_pos * m_scaler + Point<double>(width / 10, height / 2);
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
		Point<double> pt(robot_poses[i].pos.x * scaler + width / 10,
		                 robot_poses[i].pos.y * scaler + height / 2);

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
