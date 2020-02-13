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
	message_filters::Subscriber<sensor_msgs::Image> r_img_sub(
	    n, (*params).image_right, 1);
	message_filters::Subscriber<sensor_msgs::Image> d_img_sub(
	    n, (*params).image_depth, 1);

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
	                                                  sensor_msgs::Image,
                                                    sensor_msgs::Image>
	    MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000),
	                                                 l_img_sub, r_img_sub, d_img_sub);
	sync.registerCallback(boost::bind(&Mapper::imageListener, this, _1, _2, _3));

#ifdef DEBUG
	image_transport::ImageTransport it(n);
	r_img_publisher   = it.advertise("detection_right/image_raw", 1);
	l_img_publisher   = it.advertise("detection_left/image_raw", 1);
	matches_publisher = it.advertise("matches/image_raw", 1);
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
                           const sensor_msgs::ImageConstPtr& msg_right,
                           const sensor_msgs::ImageConstPtr& msg_depth)
{
	if ((*msg_left).header.stamp == (*msg_right).header.stamp) {
		std::vector<coral::DetectionCandidate> left_res  = detect(msg_left);
		std::vector<coral::DetectionCandidate> right_res = detect(msg_right);

		/* process left image results */
		std::vector<Point<double>> left_det;
		for (auto result : left_res) {
			double xmin = result.corners.xmin * (*msg_left).height;
			double ymin = result.corners.ymin * (*msg_left).width;
			double xmax = result.corners.xmax * (*msg_left).height;
			double ymax = result.corners.ymax * (*msg_left).width;

			Point<double> tmp((ymin + ymax) / 2, (xmin + xmax) / 2);
			if (result.label == 0) {
				left_det.push_back(tmp);
			}
		}

		/* process right image results */
		std::vector<Point<double>> right_det;
		for (auto result : right_res) {
			double xmin = result.corners.xmin * (*msg_right).height;
			double ymin = result.corners.ymin * (*msg_right).width;
			double xmax = result.corners.xmax * (*msg_right).height;
			double ymax = result.corners.ymax * (*msg_right).width;

			Point<double> tmp((ymin + ymax) / 2, (xmin + xmax) / 2);
			if (result.label == 0) {
				right_det.push_back(tmp);
			}
		}

		matches.clear();
		matches = (*lprocessor).matchLandmarks(left_det, right_det);

#ifdef DEBUG
		cv::Mat left_bboxes;
		cv::Mat right_bboxes;
		showBBoxes(msg_left, left_bboxes, left_res);
		showBBoxes(msg_right, right_bboxes, right_res);

		sensor_msgs::ImagePtr left_det_img =
		    cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_bboxes)
		        .toImageMsg();
		l_img_publisher.publish(left_det_img);

		sensor_msgs::ImagePtr right_det_img =
		    cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_bboxes)
		        .toImageMsg();
		r_img_publisher.publish(right_det_img);

		/* if (matches.size() > 0) */
		/* 	showMatching(left_bboxes, right_bboxes); */
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
