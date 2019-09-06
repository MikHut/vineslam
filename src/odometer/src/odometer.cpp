#include "../include/odometer.hpp"

Odometer::Odometer()
{
  ros::NodeHandle local_nh("~");

  /* read launch file parameters */
  local_nh.getParam("/odometer/img_width", width);
  local_nh.getParam("/odometer/img_height", heigth);
  local_nh.getParam("/odometer/grid_resolution", resolution);

#ifdef VISUALIZE
  image_transport::ImageTransport it(local_nh);
  grid_pub = it.advertise("/odometer/grid", 1);

  last_grid = cv::Mat(resolution * 100, resolution * 100, CV_8UC1,
		      cv::Scalar(255, 255, 255));
#endif

  processor = new LandmarkProcessor(width, heigth, resolution);
}

void Odometer::boxListener(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
  /* center of mass calculation */
  std::vector<Point<int>> center_of_mass;
  for (auto i : (*msg).bounding_boxes) {
    Point<int> tmp((i.xmin + i.xmax) / 2, (i.ymin + i.ymax) / 2);
    center_of_mass.push_back(tmp);
  }

  /* grid design and publication */
  (*processor).updatePoses(center_of_mass);
  cv::Mat grid = (*processor).buildGrid();

#ifdef VISUALIZE
  cv::Mat concat;
  cv::hconcat(grid, last_grid, concat);
  sensor_msgs::ImagePtr img =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", concat).toImageMsg();
  grid_pub.publish(img);

  last_grid = grid;
#endif
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometer");

  ros::NodeHandle nh;

  Odometer*       odometer = new Odometer();
  ros::Subscriber box_sub  = nh.subscribe("/darknet_ros/bounding_boxes", 1000,
					  &Odometer::boxListener, odometer);

  ros::spin();
  return 0;
}
