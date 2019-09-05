// c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>


// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>


// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

geometry_msgs::Twist velocity;

class p_supervisor {

private:


  ros::NodeHandle nh_;

  ros::Subscriber boundindBoxes;
  ros::Publisher supervisor_vel;



public:

  p_supervisor(ros::NodeHandle &nh) {

    nh_ = nh;

    boundindBoxes = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 100, &p_supervisor::yolo_listener, this);
    supervisor_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10, false);

  }





  void yolo_listener(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes) {

    for (int32_t i = 0; i < boxes->bounding_boxes.size(); i++) {

      if (!boxes->bounding_boxes[i].Class.compare("person")) {

        ROS_WARN("PERSON DETECTED");

        velocity.linear.x = 0;
        velocity.linear.y = 0;
        velocity.linear.z = 0;
        velocity.angular.x = 0;
        velocity.angular.y = 0;
        velocity.angular.z = 0;

        supervisor_vel.publish(velocity);

        ros::Duration(0.1).sleep();

      }
    }
  }


};


int main(int argc, char** argv) {
  //init the ROS node
  ros::init(argc, argv, "person_supervisor");

  ros::NodeHandle nh("~");
  p_supervisor supervisor_velocity(nh);

  // std::string path = ros::package::getPath("Agrob_path");
  // path.append("/Data/Map_PC2GD/map.pgm");


  // nh.param<int32_t>("angle", ths_angle, 30);


  ros::spin();

}
