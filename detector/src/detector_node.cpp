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
	l_img_publisher = it.advertise("/detection_left/image_raw", 1);
#endif
	// Publish map and particle filter
	map_publisher = n.advertise<visualization_msgs::MarkerArray>("/map_2D", 1);
	pcl_publisher = n.advertise<sensor_msgs::PointCloud2>("/map_3D", 1);
	particle_publisher = n.advertise<geometry_msgs::PoseArray>("/particles", 1);
	odom_publisher     = n.advertise<nav_msgs::Odometry>("/odometry", 1);

	// Declarate Mapper2D and Localizer objects
	localizer = new Localizer(*params);
	mapper_2d = new Mapper2D(*params);
	mapper_3d = new Mapper3D(*params);

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
	// Convert odometry msg to pose msg
	tf::Pose            pose;
	geometry_msgs::Pose odom_pose = (*msg).pose.pose;
	tf::poseMsgToTF(odom_pose, pose);

	// Check if yaw is NaN
	double yaw = tf::getYaw(pose.getRotation());
	if (yaw != yaw)
		yaw = 0;

	// If it is the first iteration - initialize the Pose
	// relative to the previous frame
	if (init == true) {
		p_odom.pos.x = (*msg).pose.pose.position.x;
		p_odom.pos.y = (*msg).pose.pose.position.y;
		p_odom.yaw   = yaw;
		odom.yaw     = yaw;
		return;
	}

	// Integrate odometry pose to convert to the map frame
	odom.pos.x += (*msg).pose.pose.position.x - p_odom.pos.x;
	odom.pos.y += (*msg).pose.pose.position.y - p_odom.pos.y;
	odom.pos.z = 0;
	odom.roll  = 0;
	odom.pitch = 0;
	odom.yaw   = yaw;

	// Create quaternion with the orientation relative
	// to the map
	tf::Quaternion q;
	q.setRPY(odom.roll, odom.pitch, odom.yaw);

	// Update odometry msg in relation to the map, to publish
	odom_.header                = (*msg).header;
	odom_.pose.pose.position.x  = odom.pos.x;
	odom_.pose.pose.position.y  = odom.pos.y;
	odom_.pose.pose.position.z  = odom.pos.z;
	odom_.pose.pose.orientation = (*msg).pose.pose.orientation;

	// Publish odometry msg
	odom_publisher.publish(odom_);

	// Save current odometry pose to use in the next iteration
	p_odom.pos.x = (*msg).pose.pose.position.x;
	p_odom.pos.y = (*msg).pose.pose.position.y;
	p_odom.yaw   = yaw;
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
			if (depth == -1)
				continue;

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
			// Initialize the localizer and get first particles distribution
			(*localizer).init(Pose<double>(0, 0, 0, 0, 0, odom.yaw));
			Pose<double> robot_pose = (*localizer).getPose();

			// Initialize the mapper_2d
			(*mapper_2d).init(robot_pose, bearings, depths, info);

			// Get first map_2d
			map_2d = (*mapper_2d).getMap();

			init = false;
		}
		else if (init == false) {
			// Execute the localization procedure
			(*localizer).process(odom, bearings, depths, map_2d);
			Pose<double>             robot_pose = (*localizer).getPose();
			geometry_msgs::PoseArray poses      = (*localizer).getPoseArray();
			tf::Transform            cam2map    = (*localizer).getTf();

			// Execute the 2D map estimation
			(*mapper_2d).process(robot_pose, bearings, depths, cam2map, info);

			// Get the curretn 2D map
			map_2d = (*mapper_2d).getMap();

			// Execute the 3D map estimation
			float* depths = (float*)(&(*msg_depth).data[0]);
			(*mapper_3d).process(depths);
			// Publish 3D point cloud
			publish3DCloud();

			// Publish the 2D map and particle filter
			publish2DMap(odom_.header, robot_pose);
			poses.header = odom_.header;
			particle_publisher.publish(poses);

			// Publish cam-to-world tf::Transform
			static tf::TransformBroadcaster br;
			br.sendTransform(
			    tf::StampedTransform(cam2map, ros::Time::now(), "map", "cam"));
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
	// Declare array with all the disparities computed
	float* depths = (float*)(&(depth_img).data[0]);

	double range_min = 0.01;
	double range_max = 10.0;

	std::vector<float> depth_array;
	for (int x = xmin; x < xmax; x++) {
		for (int y = ymin; y < ymax; y++) {
			int idx = x + depth_img.width * y;

			// Fill the depth array with the values of interest
			if (std::isfinite(depths[idx]) && depths[idx] > range_min &&
			    depths[idx] < range_max)
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

void Detector::publish3DCloud()
{
	// Get the raw point cloud to publish
	std::vector<Point<double>> in_pcl = (*mapper_3d).getPointCloud();

	// Convert the point cloud to sensor_msgs::PointCloud
	sensor_msgs::PointCloud tmp_pcl;
	tmp_pcl.header          = odom_.header;
	tmp_pcl.header.frame_id = "cam";

	for (size_t i = 0; i < in_pcl.size(); i++) {
		geometry_msgs::Point32 pt;
		pt.x = in_pcl[i].z;
		pt.y = -in_pcl[i].x;
		pt.z = -in_pcl[i].y;

		tmp_pcl.points.push_back(pt);
	}

	// Convert sensor_msgs::PointCloud to sensor_msgs::PointCloud2
	sensor_msgs::PointCloud2 out_pcl;
	sensor_msgs::convertPointCloudToPointCloud2(tmp_pcl, out_pcl);

	// Publish PointCloud
	pcl_publisher.publish(out_pcl);
}
