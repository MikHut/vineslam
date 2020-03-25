#include "../include/detector/detector_node.hpp"

void Detector::showBBoxes(const sensor_msgs::ImageConstPtr&             msg,
                          cv::Mat&                                      bboxes,
                          const std::vector<coral::DetectionCandidate>& res)
{
	std::vector<Point<double>> trunk_pos;
	for (auto result : res) {
		double xmin = result.corners.ymin * (*msg).width;
		double ymin = result.corners.xmin * (*msg).height;
		double xmax = result.corners.ymax * (*msg).width;
		double ymax = result.corners.xmax * (*msg).height;
		double xavg = (xmin + xmax) / 2;

		Point<double> tmp((xmin + xmax) / 2, (ymin + ymax) / 2);
		trunk_pos.push_back(tmp);

		if (result.score > 0.5) {
			cv::rectangle(bboxes, cv::Point(xmin, ymin), cv::Point(xmax, ymax),
			              cv::Scalar(255, 0, 0), 2);
			cv::line(bboxes, cv::Point(xavg, ymin), cv::Point(xavg, ymax),
			         cv::Scalar(0, 255, 0), 2);
		}
	}
}
