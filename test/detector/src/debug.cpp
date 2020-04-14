#include "../include/detector/detector_node.hpp"

void Detector::showBBoxes(const sensor_msgs::ImageConstPtr& left_image,
                          const sensor_msgs::ImageConstPtr& depth_image,
                          cv::Mat&                          bboxes,
                          const std::vector<coral::DetectionCandidate>& res)
{
	for (auto result : res) {
		float xmin = result.corners.ymin * (*left_image).width;
		float ymin = result.corners.xmin * (*left_image).height;
		float xmax = result.corners.ymax * (*left_image).width;
		float ymax = result.corners.xmax * (*left_image).height;
		float xavg = (xmin + xmax) / 2;

		point3D tmp((xmin + xmax) / 2, (ymin + ymax) / 2, 0.);

		if (result.score > 0.5) {
			cv::rectangle(bboxes, cv::Point(xmin, ymin), cv::Point(xmax, ymax),
			              cv::Scalar(255, 0, 0), 2);
			cv::line(bboxes, cv::Point(xavg, ymin), cv::Point(xavg, ymax),
			         cv::Scalar(0, 255, 0), 2);

		}
	}
}
