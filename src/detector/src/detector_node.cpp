#include "../include/detector/detector_node.hpp"

Detector::Detector(int argc, char** argv)
{
	ros::init(argc, argv, "detector");
	ros::NodeHandle n;

	params = new Parameters();
	loadParameters(n);
	init = true;

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

	// Odometry subscription
	ros::Subscriber odom_subscriber =
	    n.subscribe((*params).odom_topic, 1, &Detector::odomListener, this);

#ifdef DEBUG
	image_transport::ImageTransport it(n);
	l_img_publisher = it.advertise("detection_left/image_raw", 1);
#endif
	// Publish map and particle filter
	map_publisher      = n.advertise<visualization_msgs::MarkerArray>("/map", 1);
	particle_publisher = n.advertise<geometry_msgs::PoseArray>("/particles", 1);

	// Declarate Mapper and Localizer objects
	localizer = new Localizer(*params);
	mapper    = new Mapper(*params);

	// Load NN model and labels file
	ROS_INFO("Loading NN model and label files");
	engine             = new coral::DetectionEngine((*params).model);
	input_tensor_shape = (*engine).get_input_tensor_shape();
	labels             = coral::ReadLabelFile((*params).labels);
	ROS_INFO("Done");

	// Ros spin
	run();
}

void Detector::run()
{
	ros::spin();
	std::cout << "Ros shutdown, saving the map." << std::endl;
}

void Detector::odomListener(const nav_msgs::OdometryConstPtr& msg)
{
	tf::Pose            pose;
	geometry_msgs::Pose odom_pose = (*msg).pose.pose;
	tf::poseMsgToTF(odom_pose, pose);

	double yaw = tf::getYaw(pose.getRotation());
	if (yaw != yaw)
		yaw = 0;

	odom.pos.x = (*msg).pose.pose.position.x;
	odom.pos.y = (*msg).pose.pose.position.y;
	odom.pos.z = 0;
	odom.roll  = 0;
	odom.pitch = 0;
	odom.yaw   = yaw;

	odom_ = *msg;
}

void Detector::imageListener(const sensor_msgs::ImageConstPtr& msg_left,
                             const sensor_msgs::ImageConstPtr& msg_depth)
{
#ifdef DEBUG
	cv::Mat left_bboxes =
	    cv_bridge::toCvShare(msg_left, sensor_msgs::image_encodings::BGR8)->image;
#endif

	std::vector<double>       bearings;
	std::vector<double>       depths;
	std::vector<SemanticInfo> info;

	if ((*msg_left).header.stamp == (*msg_depth).header.stamp) {
		std::vector<coral::DetectionCandidate> left_res = detect(msg_left);

		// Process left image results - calculate bearings and depths
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

			bearings.push_back(columnToTheta(tmp.x));
			depths.push_back(depth);
			info.push_back(fillSemanticInfo(result.label));

#ifdef DEBUG
			// Draw depths on debug detection image
			if (depth > 0) {
				std::string s = boost::lexical_cast<std::string>(depth);
				cv::putText(left_bboxes, s, cv::Point((xmin + xmax) / 2, ymin - 10),
				            cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 0));
			}
#endif
		}

		if (init == true && bearings.size() > 1) {
			// Initialize the localizer and the mapper
			first_odom = odom;
			(*localizer).init(odom - first_odom);
			(*mapper).init(odom - first_odom, bearings, depths, info);
			map  = (*mapper).getMap();
			init = false;
		}
		else {
			// Execute the localization procedure
			(*localizer).process(odom - first_odom, bearings, depths, map);
			Pose<double>             robot_pose = (*localizer).getPose();
			geometry_msgs::PoseArray poses      = (*localizer).getPoseArray();
			tf::Transform            cam2world  = (*localizer).getTf();

			// Execute the map estimation
			(*mapper).process(robot_pose, bearings, depths, cam2world, info);
			// Get the curretn map
			map = (*mapper).getMap();
			// Publish the map and particle filter
			publishMap(odom_.header);
			poses.header = odom_.header;
			particle_publisher.publish(poses);
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

	auto results =
	    (*engine).DetectWithInputTensor(input_tensor, (*params).min_score, 50);

	return results;
}

double Detector::computeDepth(const sensor_msgs::Image& depth_img,
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

	// compute median of all observations
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
