#include "../include/mapper/mapper_node.hpp"

Mapper::Mapper(int argc, char** argv)
{
	ros::init(argc, argv, "mapper");
	ros::NodeHandle n;

	params = new Parameters();
	loadParameters(n);
	init = true;

	message_filters::Subscriber<sensor_msgs::Image> l_img_sub(
	    n, (*params).image_left, 1);
	message_filters::Subscriber<sensor_msgs::Image> d_img_sub(
	    n, (*params).image_depth, 1);

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
	                                                  sensor_msgs::Image>
	    MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), l_img_sub,
	                                                 d_img_sub);
	sync.registerCallback(boost::bind(&Mapper::imageListener, this, _1, _2));

#ifdef DEBUG
	image_transport::ImageTransport it(n);
	l_img_publisher = it.advertise("detection_left/image_raw", 1);
#endif

	ROS_INFO("Loading NN model and label files");
	engine             = new coral::DetectionEngine((*params).model);
	input_tensor_shape = (*engine).get_input_tensor_shape();
	labels             = coral::ReadLabelFile((*params).labels);
	ROS_INFO("Done");

	lprocessor = new LandmarkProcessor(*params);

	run();
}

void Mapper::run()
{
	ros::spin();
	std::cout << "Ros shutdown, saving the map." << std::endl;
}

void Mapper::imageListener(const sensor_msgs::ImageConstPtr& msg_left,
                           const sensor_msgs::ImageConstPtr& msg_depth)
{
#ifdef DEBUG
	cv::Mat left_bboxes =
	    cv_bridge::toCvShare(msg_left, sensor_msgs::image_encodings::BGR8)->image;
#endif

	if ((*msg_left).header.stamp == (*msg_depth).header.stamp) {
		std::vector<coral::DetectionCandidate> left_res = detect(msg_left);

		/* process left image results */
		std::vector<Point<double>> left_det;
		for (auto result : left_res) {
			double xmin = result.corners.ymin * (*msg_left).width;
			double ymin = result.corners.xmin * (*msg_left).height;
			double xmax = result.corners.ymax * (*msg_left).width;
			double ymax = result.corners.xmax * (*msg_left).height;

			Point<double> tmp((xmin + xmax) / 2, (ymin + ymax) / 2);
			if (result.label == 0) {
				left_det.push_back(tmp);
			}

			float depth = computeDepth(*msg_depth, xmin, ymin, xmax, ymax);

#ifdef DEBUG
			if (depth > 0) {
				std::string s = boost::lexical_cast<std::string>(depth);
				cv::putText(left_bboxes, s, cv::Point((xmin + xmax) / 2, ymin - 10),
				            cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 0));
			}
#endif
		}

#ifdef DEBUG
		showBBoxes(msg_left, left_bboxes, left_res);

		sensor_msgs::ImagePtr left_det_img =
		    cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_bboxes)
		        .toImageMsg();
		l_img_publisher.publish(left_det_img);
#endif
	}
}

std::vector<coral::DetectionCandidate>
Mapper::detect(const sensor_msgs::ImageConstPtr& msg)
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

	return results;
}

double Mapper::computeDepth(const sensor_msgs::Image& depth_img,
                            const int& xmin, const int& ymin, const int& xmax,
                            const int& ymax)
{
	float* depths = (float*)(&(depth_img).data[0]);

	std::vector<float> depth_array;
	for (int x = xmin; x < xmax; x++) {
		for (int y = ymin; y < ymax; y++) {
			int idx = x + depth_img.width * y;

			if (std::isfinite(depths[idx]) && depths[idx] > 0)
				depth_array.push_back(depths[idx]);
		}
	}

	/* compute median of all observations */
	size_t n_depths = depth_array.size();
	if (n_depths > 0) {
		std::sort(depth_array.begin(), depth_array.end());
		if (n_depths % 2 == 0)
			return (depth_array[n_depths / 2 - 1] + depth_array[n_depths / 2]) / 2;
		else
			return depth_array[n_depths / 2];
	}
	else
		return -1;
}
