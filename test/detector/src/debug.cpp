#include "../include/detector/detector_node.hpp"

void Detector::showBBoxes(const sensor_msgs::ImageConstPtr& rgb_image,
                          const sensor_msgs::ImageConstPtr& depth_image,
                          cv::Mat&                          bboxes,
                          const std::vector<coral::DetectionCandidate>& res)
{
	std::vector<Point<double>> trunk_pos;
	for (auto result : res) {
		double xmin = result.corners.ymin * (*rgb_image).width;
		double ymin = result.corners.xmin * (*rgb_image).height;
		double xmax = result.corners.ymax * (*rgb_image).width;
		double ymax = result.corners.xmax * (*rgb_image).height;
		double xavg = (xmin + xmax) / 2;

		Point<double> tmp((xmin + xmax) / 2, (ymin + ymax) / 2);
		trunk_pos.push_back(tmp);

		if (result.score > 0.5) {
			cv::rectangle(bboxes, cv::Point(xmin, ymin), cv::Point(xmax, ymax),
			              cv::Scalar(255, 0, 0), 2);
			cv::line(bboxes, cv::Point(xavg, ymin), cv::Point(xavg, ymax),
			         cv::Scalar(0, 255, 0), 2);

			Point<double> index =
			    computeDepth(*depth_image, (int)xmin, (int)ymin, (int)xmax, (int)ymax);
			cv::circle(bboxes, cv::Point(index.x, index.y), 5, cv::Scalar(0, 0, 0), 5);
		}
	}
}

Point<double> Detector::computeDepth(const sensor_msgs::Image& depth_img,
                                     const int& xmin, const int& ymin,
                                     const int& xmax, const int& ymax)
{
	// Declare array with all the disparities computed
	float* depths = (float*)(&(depth_img).data[0]);

	double range_min = 0.01;
	double range_max = 10.0;

  std::map<float, Point<double>> depth_map;

	for (int x = xmin; x < xmax; x++) {
		for (int y = ymin; y < ymax; y++) {
			int idx = x + depth_img.width * y;

			// Fill the depth array with the values of interest
			if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
			    depths[idx] < range_max) {
        depth_map[depths[idx]] = Point<double>(x,y);
			}
		}
	}
  return depth_map.begin()->second;
}
