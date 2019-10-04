#include "qnode.hpp"

QNode::QNode(int argc, char** argv, const std::string& node_name)
    : init_argc(argc), init_argv(argv), node_name(node_name)
{
}

QNode::~QNode()
{
	ros::shutdown();
}

bool QNode::init()
{
	ros::init(init_argc, init_argv, node_name);
	if (!ros::master::check()) {
		return false;
	}
	ros::start();

  rosCommsInit();

	start();
	return true;
}
