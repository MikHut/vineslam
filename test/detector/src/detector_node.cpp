#include "../include/detector/detector_node.hpp"

Detector::Detector(int argc, char** argv)
{
	ros::init(argc, argv, "detector");
	ros::NodeHandle n;

	// Load ROS parameters
	params = new Parameters();
	loadParameters(n);

	// Left and depth images subscription
	message_filters::Subscriber<sensor_msgs::Image> l_img_sub(
	    n, (*params).image_left, 1);
	message_filters::Subscriber<sensor_msgs::Image> d_img_sub(
	    n, (*params).image_depth, 1);

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
	                                                  sensor_msgs::Image>
	                                            MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), l_img_sub,
	                                                 d_img_sub);
	sync.registerCallback(boost::bind(&Detector::imageListener, this, _1, _2));

	// Declare bboxes and image publishers
	bbox_publisher = n.advertise<vision_msgs::Detection2DArray>("/detections", 1);
	image_transport::ImageTransport it(n);
	l_img_publisher = it.advertise("/detection_left/image_raw", 1);

	// Load NN model and labels file
	ROS_INFO("Loading NN model and label files");
	engine             = new coral::DetectionEngine((*params).model);
	input_tensor_shape = (*engine).get_input_tensor_shape();
	labels             = coral::ReadLabelFile((*params).labels);
	ROS_INFO("Done");

	// Ros spin
	ros::spin();
}

void Detector::imageListener(const sensor_msgs::ImageConstPtr& msg_left,
                             const sensor_msgs::ImageConstPtr& msg_depth)
{
	cv::Mat left_bboxes =
	    cv_bridge::toCvShare(msg_left, sensor_msgs::image_encodings::BGR8)->image;

	// Declare the detection array
	vision_msgs::Detection2DArray dets;
	dets.header = (*msg_depth).header;

	std::vector<coral::DetectionCandidate> left_res = detect(msg_left);

	// Process left image results - calculate bearings and depths
	std::vector<Point<double>> left_det;
	for (auto result : left_res) {
		// Calculate bounding box image coordinates
		double xmin = result.corners.ymin * (*msg_left).width;
		double ymin = result.corners.xmin * (*msg_left).height;
		double xmax = result.corners.ymax * (*msg_left).width;
		double ymax = result.corners.xmax * (*msg_left).height;

    // AUX
    Point<double> tmp((xmin + xmax) / 2, (ymin + ymax) / 2);

		// Declare detection information
		vision_msgs::ObjectHypothesisWithPose det_info;
		det_info.id    = result.label;
		det_info.score = result.score;
		// Declare bounding box location
		vision_msgs::BoundingBox2D bbox;
		bbox.center.x = (xmin + xmax) / 2;
		bbox.center.y = (ymin + ymax) / 2;
		bbox.size_x   = (xmax - xmin);
		bbox.size_y   = (ymax - ymin);
		// Save detection
		vision_msgs::Detection2D m_det;
		m_det.results.push_back(det_info);
		m_det.bbox            = bbox;
		m_det.header          = (*msg_depth).header;
		m_det.header.frame_id = "detection";
		// Push back to detection array
		dets.detections.push_back(m_det);
	}
	// Publish bounding box detections
	bbox_publisher.publish(dets);

	// Draw bounding boxes on incoming image
	showBBoxes(msg_left, left_bboxes, left_res);

	// Publish image with corresponding bboxes
	sensor_msgs::ImagePtr left_det_img =
	    cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_bboxes).toImageMsg();
	l_img_publisher.publish(left_det_img);
}

std::vector<coral::DetectionCandidate>
Detector::detect(const sensor_msgs::ImageConstPtr& msg)
{
	// Convert input image to BGR
	cv_bridge::CvImageConstPtr cv_ptr =
	    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	sensor_msgs::ImagePtr tmp =
	    cv_bridge::CvImage(std_msgs::Header(), "bgr8", (*cv_ptr).image)
	        .toImageMsg();

	// ROS image to std::vector<uint8_t>
	std::vector<uint8_t> in_image;
	for (int i = 0; i < (*tmp).step * (*msg).height; i++)
		in_image.push_back((*tmp).data[i]);

	// Trunk detection
	std::vector<uint8_t> input_tensor = coral::GetInputFromImage(
	    in_image,
	    {input_tensor_shape[1], input_tensor_shape[2], input_tensor_shape[3]},
	    {(*msg).height, (*msg).width, 3});

	auto results = (*engine).DetectWithInputTensor(input_tensor, 0.5, 50);

	return results;
}
