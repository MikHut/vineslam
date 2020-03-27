#include "../include/wildSLAM_ros.hpp"

wildSLAM_ros::SLAMNode::SLAMNode(int argc, char** argv)
{
	// Initialize ROS node
	ros::init(argc, argv, "SLAMNode");
	ros::NodeHandle nh;

	// Set initialize flag default values
	init = true;

	// Synchronize subscribers of both topics
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/image", 1);
	message_filters::Subscriber<vision_msgs::Detection2DArray> detections_sub(
	    nh, "/detections", 1);
	message_filters::TimeSynchronizer<sensor_msgs::Image,
	                                  vision_msgs::Detection2DArray>
	    sync(image_sub, detections_sub, 10);
	sync.registerCallback(boost::bind(&SLAMNode::callbackFct, this, _1, _2));

	// Odometry subscription
	ros::Subscriber odom_subscriber =
	    nh.subscribe("/odom", 1, &SLAMNode::odomListener, this);

	// Publish maps and particle filter
	map2D_publisher =
	    nh.advertise<visualization_msgs::MarkerArray>("/wildSLAM/map2D", 1);
	map3D_raw_publisher=
	    nh.advertise<visualization_msgs::MarkerArray>("/wildSLAM/map3D/raw", 1);
	map3D_trunk_publisher =
	    nh.advertise<visualization_msgs::MarkerArray>("/wildSLAM/map3D/trunks", 1);
	particle_publisher =
	    nh.advertise<geometry_msgs::PoseArray>("/wildSLAM/particles", 1);

	// Load params
	std::string config_path;
	if (nh.getParam("/wildSLAM_ros/SLAMNode/config_path", config_path) == false) {
		ROS_ERROR("/config_path parameter not found. Shutting down...");
		return;
	}

	// Load input numeric parameters
	YAML::Node config = YAML::LoadFile(config_path);
	img_width         = config["camera_info"]["img_width"].as<double>();
	h_fov             = config["camera_info"]["h_fov"].as<double>() * PI / 180;

	// Declarate the Mappers and Localizer objects
	localizer = new Localizer(config_path);
	mapper2D  = new Mapper2D(config_path);
	mapper3D  = new Mapper3D(config_path);

	ros::spin();
	ROS_INFO("ROS shutting down...");
}
