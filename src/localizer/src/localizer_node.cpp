#include "../include/localizer/localizer_node.hpp"

Localizer::Localizer(int argc, char** argv)
{
	ros::init(argc, argv, "localizer");

	ros::NodeHandle n;
	params = new Parameters();
	loadParameters(n);

  loadLandmarks((*params).map_file);

	img_subscriber =
	    n.subscribe((*params).image_topic, 1, &Localizer::imageListener, this);

	ros::spin();
}

void Localizer::imageListener(const sensor_msgs::ImageConstPtr& msg)
{
}
