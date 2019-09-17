#include "../include/odometer.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometer");
	ros::NodeHandle nh;

	Odometer* odometer = new Odometer();

	ros::Subscriber box_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1000,
	                                       &Odometer::boxListener, odometer);

#ifdef VISUALIZE
	image_transport::ImageTransport it(nh);
	(*odometer).grid_pub      = it.advertise("/odometer/debug", 1);
	(*odometer).matches_pub   = it.advertise("/odometer/matches_image", 1);
	(*odometer).particles_pub = it.advertise("/odometer/particles_image", 1);

	image_transport::Subscriber bb_sub = it.subscribe(
	    "/darknet_ros/detection_image", 1, &Odometer::imageListener, odometer);
#endif

	ros::spin();
	return 0;
}
