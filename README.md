# wildSLAM

This package contains a SLAM approach dedicated to vineyards.
The estimator receives a disparity image and a set of bounding boxes locating objects in
an image, and outputs a 2D-feature map, a 3D occupancy grid map, and the robot pose.

## Dependencies

### Standalone wildSLAM package

* **ROS** - [melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) is the recomended version
* **octomap** - `sudo apt install ros-melodic-octomap*`
* **vision_msgs** - `sudo apt install ros-melodic-vision-msgs`

### Test package

To use the test package, the dependencies of the Edge TPU API have also to be installed.
All the dependencies are listed [here](https://gitlab.inesctec.pt/agrob/coral-deeplearning-ros), as well as the desired installation procedure.

## Installation

To install simply clone and compile the project.
```
cd src/<user>/catkin_ws
git clone https://gitlab.inesctec.pt/agrob/agrobvslam
catkin_make -DCMAKE_BUILD_TYPE:=Release
```

## ROS structure

### Subscribed topics

* **/image** ([sensor_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)) - the disparity map
* **/detections**
  ([vision_msgs::Detection2DArray](http://docs.ros.org/api/vision_msgs/html/msg/Detection2DArray.html)) - the object detections
* **/odom**
  ([nav_msgs::Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)) - the robot wheel Odometry

### Published topics

* **/wildSLAM/map2D**
  ([visualization_msgs::MarkerArray](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html)) - the 2D map
* **/wildSLAM/map3D/raw**
  ([sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) - the raw 3D map
* **/wildSLAM/particles**
  ([geometry_msgs::PoseArray](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseArray.html)) - the robot pose

### Transforms

* **cam2map** ([tf::Transform](http://docs.ros.org/jade/api/tf/html/c++/classtf_1_1Transform.html)) - camera to map dynamic transformation encoding the robot pose
  relatively to the map

## How to run

To run wildSLAM and visualize the 3D map using OctoMap create a launch file with the following format

```
  <!-- slam node -->
  <node pkg="wildSLAM_ros" type="wildSLAM_ros" name="wildSLAM_ros" output="screen">
    <remap from="/image" to="/zed/zed_node/depth/depth_registered" />
    <remap from="/detections" to="detections" />
    <remap from="/odom" to="/husky_velocity_controller/odom" />
    
    <param name="/SLAMNode/config_path" value="$(find wildSLAM)/config/setup.yaml" />
  </node>

  <!-- octomap_server node -->
  <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
    <param name="resolution" value="0.05" />
    <param name="frame_id" type="string" value="map" />
    <param name="sensor_model/max_range" value="20.0" />
    <remap from="cloud_in" to="/wildSLAM/map3D/raw" />   
   </node>
```

## How to test

If you want to test the system we provide a test case that uses a Coral USB Accelerator in
the `test` folder.
This folder contains:
  * An Edge TPU API that performs object detection
  * A `detector` node that uses this API and publishes the detections in the desired format

We also provide a model suitable for vine trunk detection in the `test/edgetpu_cpp/model/`
dir, that is loaded by the `detector` launch file.

You can download several rosbags to test the system
[here](www.vcriis01.inesctec.pt/datasets/DataSet/Romovi/aveleda_2020-01-16-11-agrob17.zip). 
