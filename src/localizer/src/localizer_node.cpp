#include "../include/localizer/localizer_node.hpp"

Localizer::Localizer(int argc, char** argv)
{
	ros::init(argc, argv, "localizer");

	ros::NodeHandle n;
	params = new Parameters();
	loadParameters(n);

  loadLandmarks((*params).map_file);

	img_subscriber =
	    n.subscribe((*params).image_topic, 1, &Localizer::imageListener, this);

  /* edgetpu model/label/input tensor load */
	engine             = new coral::DetectionEngine((*params).model);
	input_tensor_shape = (*engine).get_input_tensor_shape();
	labels             = coral::ReadLabelFile((*params).labels);

  pfilter = new ParticleFilter(*params);
  (*pfilter).init(landmarks);

	ros::spin();
}

void Localizer::imageListener(const sensor_msgs::ImageConstPtr& msg)
{
	/* Convert input image to BGR */
	cv_bridge::CvImageConstPtr cv_ptr =
	    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	sensor_msgs::ImagePtr tmp =
	    cv_bridge::CvImage(std_msgs::Header(), "bgr8", (*cv_ptr).image)
	        .toImageMsg();

	/* ROS image to std::vector<uint8_t> */
	std::vector<uint8_t> in_image;
	for (int i = 0; i < (*tmp).step * (*msg).height; i++)
		in_image.push_back((*tmp).data[i]);

	/* trunk detection */
	std::vector<uint8_t> input_tensor = coral::GetInputFromImage(
	    in_image,
	    {input_tensor_shape[1], input_tensor_shape[2], input_tensor_shape[3]},
	    {(*msg).height, (*msg).width, 3});

	auto results =
	    (*engine).DetectWithInputTensor(input_tensor, (*params).min_score, 50);

	/* process results - calculate trunk Center of Mass */
	std::vector<Point<double>> trunk_pos;
	for (auto result : results) {
		double xmin = result.corners.xmin * (*msg).height;
		double ymin = result.corners.ymin * (*msg).width;
		double xmax = result.corners.xmax * (*msg).height;
		double ymax = result.corners.ymax * (*msg).width;

		Point<double> tmp((ymin + ymax) / 2, (xmin + xmax) / 2);
		trunk_pos.push_back(tmp);
	}

  (*pfilter).process(trunk_pos);
}
