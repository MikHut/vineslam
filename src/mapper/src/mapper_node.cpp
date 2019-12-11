#include "../include/mapper/mapper_node.hpp"

Mapper::Mapper(int argc, char** argv)
{
	ros::init(argc, argv, "mapper");
	ros::NodeHandle n;

	params = new Parameters();
	loadParameters(n);
	lprocessor = new LandmarkProcessor(*params);
	estimator  = new Estimator(*params, lprocessor);
	init       = true;

	pose_subscriber =
	    n.subscribe((*params).pose_topic, 1, &Mapper::poseListener, this);
	img_subscriber =
	    n.subscribe((*params).image_topic, 1, &Mapper::imageListener, this);

#ifdef DEBUG
	image_transport::ImageTransport it(n);
	img_publisher     = it.advertise("detection/image_raw", 1);
	matches_publisher = it.advertise("matches/image_raw", 1);
#endif
	marker_pub = n.advertise<visualization_msgs::MarkerArray>("/map", 1);

	ROS_INFO("Loading NN model and label files");
	engine             = new coral::DetectionEngine((*params).model);
	input_tensor_shape = (*engine).get_input_tensor_shape();
	labels             = coral::ReadLabelFile((*params).labels);
	ROS_INFO("Done");

	initMarker();
	run();
}

void Mapper::run()
{
	ros::spin();
	std::cout << "Ros shutdown, saving the map." << std::endl;
	saveMap();
}

void Mapper::poseListener(const geometry_msgs::PoseStampedConstPtr& msg)
{
	slam_pose = (*msg).pose;
}

void Mapper::imageListener(const sensor_msgs::ImageConstPtr& msg)
{
	tf::Pose pose;
	tf::poseMsgToTF(slam_pose, pose);
	double yaw = tf::getYaw(pose.getRotation());
	if (yaw != yaw) // check if it is NaN
		yaw = 0;

	all_poses.push_back(
	    Pose<double>(slam_pose.position.x, slam_pose.position.y, yaw));

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
		if (result.label == 0)
			trunk_pos.push_back(tmp);
	}

	Point<double>    r_pos(slam_pose.position.x, slam_pose.position.y);
	std::vector<int> index;
	if (init == false) {
		(*lprocessor).updateDetections(trunk_pos);
		(*lprocessor).matchLandmarks(all_poses.size(), r_pos, index);

		(*estimator).process(all_poses, index);
	}
	else {
		for (size_t i = 0; i < trunk_pos.size(); i++) {
			(*lprocessor).landmarks.push_back(Landmark<double>(i, trunk_pos[i]));
			(*lprocessor).landmarks[i].ptr.push_back(0);

			if ((*params).type == "pf") {
				bool side = (trunk_pos[i].x < (*params).width / 2) ? 1 : 0;
				(*lprocessor).pf.push_back(PF(side, *params));
			}
			else {
				Eigen::MatrixXd P0(2, 2);
				P0 << 2, 0, 0, 2;
				Line<double>  l = (*lprocessor).computeLine(trunk_pos[i]);
				Point<double> X0(slam_pose.position.x, l.getY(slam_pose.position.x));
				KF            kf_(P0, X0.eig(), *params);
				(*lprocessor).kf.push_back(kf_);
			}
		}
#ifdef DEBUG
		p_image = cv::Mat((*params).width, (*params).height, CV_8UC1,
		                  cv::Scalar(255, 255, 255));
		c_image = p_image;
#endif
		init = false;
	}

	visualization_msgs::MarkerArray marker_array;
	for (size_t i = 0; i < (*lprocessor).landmarks.size(); i++) {
		Landmark<double> l = (*lprocessor).landmarks[i];
		if (l.stdev.x > 0.1 || l.stdev.y > 0.1)
			continue;

		marker.id              = i;
		marker.header          = (*msg).header;
		marker.header.frame_id = "map";
		marker.pose.position.x = l.world_pos.x;
		marker.pose.position.y = l.world_pos.y;
		marker.pose.position.z = 0;

		marker_array.markers.push_back(marker);
	}
	marker_pub.publish(marker_array);

#ifdef DEBUG
	cv::Mat bboxes;
	showBBoxes(msg, bboxes, results);
	showMatching(bboxes);
#endif
}

void Mapper::saveMap()
{
	std::ofstream map_file;
	map_file.open("/home/andre-criis/map.txt");

	for (size_t i = 0; i < (*lprocessor).landmarks.size(); i++) {
		Point<double> pt = (*lprocessor).landmarks[i].world_pos;
		map_file << i << " " << pt.x << " " << pt.y << "\n";
	}

	map_file.close();
}
