# vineslam

This package contains a SLAM approach dedicated to vineyards.
The estimator receives a disparity image and a set of bounding boxes locating objects in
an image, and outputs a 2D-feature map, a 3D occupancy grid map, and the robot pose.

## Installation

Follow the steps described in the [installation file](installation.md).

## ROS structure

### Subscribed topics

* **/image** ([sensor_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)) - the disparity map
* **/detections**
  ([vision_msgs::Detection2DArray](http://docs.ros.org/api/vision_msgs/html/msg/Detection2DArray.html)) - the object detections
* **/odom**
  ([nav_msgs::Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)) - the robot wheel Odometry

### Published topics

* **/vineslam/map2D**
  ([visualization_msgs::MarkerArray](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html)) - the 2D map
* **/vineslam/map3D/raw**
  ([sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) - the raw 3D map
* **/vineslam/map3D/trunks**
  ([sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) - the trunks 3D map
* **/vineslam/particles**
  ([geometry_msgs::PoseArray](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseArray.html)) - the robot pose

### Transforms

TODO

### Parameters

NOT FINISHED

* **camera_info**:
  * baseline - stereo camera baseline (meters)
  * delta_d - stereo matcher disparity error (pixels)
  * h_fov - stereo camera horizontal field of view (degrees)
  * cam_pitch - static camera inclination (degrees)
  * cam_height - static camera height (meters)
  * img_width - image width (pixels)
  * img_height - image height (pixels)
  * fx - horizontal focal length (pixels)
  * fy - vertical focal length (pixels)
  * cx - x coordinate of image principal point (pixels)
  * cy - y coordinate of image principal point (pixels)

* **pf**:
  * n_particles - number of particles of the particle filter
  * alpha_{1,2,3,4} - particle filter constants to spread and inovate the particles on the
    predict step.

## How to run

TODO

## Some results

![](./docs/res1.gif)
