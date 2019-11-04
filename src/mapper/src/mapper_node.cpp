#include "../include/mapper/mapper_node.hpp"
#include <boost/lexical_cast.hpp>
#include <fstream>

Mapper::Mapper(int argc, char** argv) : QNode(argc, argv, "mapper") {}

void Mapper::rosCommsInit()
{
	ros::NodeHandle n;

	params = new Parameters();
	loadParameters(n);
	lprocessor = new LandmarkProcessor(*params);
	estimator  = new Estimator(*params, *lprocessor);
	init       = true;

	pose_subscriber =
	    n.subscribe("/slam_out_pose", 1, &Mapper::poseListener, this);
	img_subscriber =
	    n.subscribe("/camera/image", 1, &Mapper::imageListener, this);

	image_transport::ImageTransport it(n);
#ifdef DEBUG
	img_publisher = it.advertise("detection/image_raw", 1);
#endif

	engine             = new coral::DetectionEngine((*params).model);
	input_tensor_shape = (*engine).get_input_tensor_shape();
	labels             = coral::ReadLabelFile((*params).labels);
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

	/* ROS image to std::vector<uint8_t> */
	cv_bridge::CvImageConstPtr cv_ptr =
	    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	cv::Mat              bboxes = (*cv_ptr).image;
	std::vector<uint8_t> in_image;
	for (int i = 0; i < (*msg).step * (*msg).height; i++)
		in_image.push_back((*msg).data[i]);

	/* trunk detection */
	std::vector<uint8_t> input_tensor = coral::GetInputFromImage(
	    in_image,
	    {input_tensor_shape[1], input_tensor_shape[2], input_tensor_shape[3]});

	auto results = (*engine).DetectWithInputTensor(input_tensor, 0.1, 10);
	for (auto result : results) {
#ifdef DEBUG
		double xmin = result.corners.xmin * (*msg).height;
		double ymin = result.corners.ymin * (*msg).width;
		double xmax = result.corners.xmax * (*msg).height;
		double ymax = result.corners.ymax * (*msg).width;
		double yavg = (ymin + ymax) / 2;

		cv::rectangle(bboxes, cv::Point(ymin, xmin), cv::Point(ymax, xmax),
		              cv::Scalar(255, 0, 0), 3);
		cv::line(bboxes, cv::Point(yavg, xmin), cv::Point(yavg, xmax),
		         cv::Scalar(0, 255, 0), 3);
#endif
	}

	last_pose.pos.x = slam_pose.position.x;
	last_pose.pos.y = slam_pose.position.y;
	last_pose.theta = yaw;

#ifdef DEBUG
	sensor_msgs::ImagePtr detection_img =
	    cv_bridge::CvImage(std_msgs::Header(), "bgr8", bboxes).toImageMsg();
	img_publisher.publish(detection_img);
#endif
}

void Mapper::constructMap()
{
	(*estimator).process((*lprocessor).landmarks, all_poses);
}

const cv::Mat Mapper::exportMap()
{
	return (*estimator).map;
}

void Mapper::retrieveLog(std::string& log)
{
	std::ostringstream tmp;
	for (size_t i = 0; i < (*lprocessor).landmarks.size(); i++) {
		tmp << "Landmark " << i << " tracked "
		    << (*lprocessor).landmarks[i].image_pos.size() << " times.";
		tmp << "\n";
	}

	log = tmp.str();
}
