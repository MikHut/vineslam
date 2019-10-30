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
	    n.subscribe("/slam_out_pose", 1000, &Mapper::poseListener, this);
	img_subscriber = n.subscribe("/camera/image", 1000,
	                             &Mapper::imageListener, this);

	coral::DetectionEngine engine((*params).model);
  std::vector<int> input_tensor_shape = engine.get_input_tensor_shape();
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


	last_pose.pos.x = slam_pose.position.x;
	last_pose.pos.y = slam_pose.position.y;
	last_pose.theta = yaw;
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
