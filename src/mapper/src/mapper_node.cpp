#include "../include/mapper/mapper_node.hpp"
#include <chrono>

Mapper::Mapper(int argc, char** argv) : QNode(argc, argv, "mapper") {}

void Mapper::rosCommsInit()
{
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

	engine             = new coral::DetectionEngine((*params).model);
	input_tensor_shape = (*engine).get_input_tensor_shape();
	labels             = coral::ReadLabelFile((*params).labels);

	Q_EMIT init_done();
}

void Mapper::run()
{
	ros::spin();
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT
	rosShutdown();
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

	all_poses.push_back(Pose<double>(slam_pose.position.x * 100,
	                                 slam_pose.position.y * 100, yaw));

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

	if (init == false) {
		(*lprocessor).updatePoses(trunk_pos);
		(*lprocessor).matchLandmarks(all_poses.size());
	}
	else {
		for (size_t i = 0; i < trunk_pos.size(); i++) {
			(*lprocessor).landmarks.push_back(Landmark<double>(i, trunk_pos[i]));
			(*lprocessor).landmarks[i].ptr.push_back(0);
		}
#ifdef DEBUG
		p_image = cv::Mat((*params).width, (*params).height, CV_8UC1,
		                  cv::Scalar(255, 255, 255));
		c_image = p_image;
#endif
		init = false;
	}

#ifdef DEBUG
	cv::Mat bboxes;
	showBBoxes(msg, bboxes, results);
	showMatching(bboxes);
#endif
}

void Mapper::constructMap(const float& scaler)
{
  (*estimator).scaler = scaler;
	(*estimator).process(all_poses);
}

const cv::Mat Mapper::exportMap()
{
	return (*estimator).map;
}

const cv::Mat Mapper::exportSingleMap(const int& id, const float& scaler)
{
	if (id > (*estimator).m_landmarks.size() - 1)
		return cv::Mat();

  (*estimator).scaler = scaler;
	(*estimator).singleDraw(all_poses, id);
	return (*estimator).single_map;
}

void Mapper::retrieveLog(std::string& log)
{
	std::ostringstream tmp;
	for (size_t i = 0; i < (*estimator).m_landmarks.size(); i++) {
		tmp << "Landmark " << i << ":\n"
		    << " -  tracked " << (*estimator).m_landmarks[i].image_pos.size()
		    << " times \n"
		    << " -  position: " << (*estimator).m_landmarks[i].world_pos
		    << " -  uncertainty: " << (*estimator).m_landmarks[i].stdev << "\n\n";
	}

	log = tmp.str();
}

void Mapper::retrieveLog(std::string& log, const int& id)
{
	std::ostringstream tmp;
	Landmark<double>   l = (*estimator).m_landmarks[id];

	tmp << "Landmark " << id << ":\n"
	    << " -  tracked " << l.image_pos.size() << " times \n"
	    << " -  position: " << l.world_pos << " -  uncertainty: " << l.stdev
	    << "\n\n";

	log = tmp.str();
}

void Mapper::saveMap()
{
  std::ofstream map_file;
  map_file.open("map.txt");

  for(size_t i = 0; i < (*estimator).m_landmarks.size(); i++) {
    Point<double> pt = (*estimator).m_landmarks[i].world_pos;
    map_file << i << " " << pt.x << " " << pt.y << "\n";
  }

  map_file.close();
}
