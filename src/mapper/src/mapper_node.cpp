#include "../include/mapper/mapper_node.hpp"

Mapper::Mapper(int argc, char** argv) : QNode(argc, argv, "mapper") {}

void Mapper::rosCommsInit()
{
	ros::NodeHandle n;
	pose_subscriber =
	    n.subscribe("/slam_out_pose", 1000, &Mapper::poseListener, this);
	box_subscriber = n.subscribe("/darknet_ros/bounding_boxes", 1000,
	                             &Mapper::boxListener, this);

	lprocessor = new LandmarkProcessor(*params);
}

void Mapper::run()
{
	ros::spin();
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT
	rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void Mapper::poseListener(const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::cout << "..." << std::endl;
	slam_pose = (*msg).pose;
}

void Mapper::boxListener(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
	/* trunks center of mass calculation */
	std::vector<Point<double>> trunk_pos;
	for (auto i : (*msg).bounding_boxes) {
		Point<double> tmp((i.xmin + i.xmax) / 2, (i.ymin + i.ymax) / 2);
		trunk_pos.push_back(tmp);
	}

	tf::Pose pose;
	tf::poseMsgToTF(slam_pose, pose);
	double yaw = tf::getYaw(pose.getRotation());
	if (yaw != yaw) // check if it is NaN
		yaw = 0;

	Pose<double> delta_pose;
	if (init == false) {
		Point<double> pt((slam_pose.position.x - last_pose.pos.x) * 100,
		                 (slam_pose.position.y - last_pose.pos.y) * 100);
		delta_pose = Pose<double>(pt, yaw - last_pose.theta);
	}
	else {
		for (size_t i = 0; i < trunk_pos.size(); i++) {
			(*lprocessor).landmarks.push_back(Landmark<double>(i, trunk_pos[i]));
			(*lprocessor).landmarks[i].r_pose.push_back(Pose<double>(0, 0, 0));
		}

		init       = false;
		delta_pose = Pose<double>(Point<double>(MEAN_X, MEAN_Y), MEAN_THETA);
	}
}

void Mapper::retrieveLog(std::string& log)
{
  std::ostringstream tmp;
  for(size_t i = 0; i < (*lprocessor).landmarks.size(); i++) {
  tmp << "Landmark " << i << ":\n";
		for (size_t j = 0; j < (*lprocessor).landmarks[i].image_pos.size(); j++) {
      tmp << " - " << (*lprocessor).landmarks[i].image_pos[j];
    }
    tmp << "\n";
  }

  log = tmp.str();
}
