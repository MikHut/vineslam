# VineSLAM ROS architecture

---

### ROS nodes

* **slam_node** - Simultaneous Localization and Mapping node
* **localization_node** - Localization-only node. Loads a map at the initialization.
* **mapping_node** - Mapping with known poses node. 

### Subscribed topics

* **"/detections_topic"**
  ([vision_msgs::Detection2DArray](http://docs.ros.org/api/vision_msgs/html/msg/Detection2DArray.html)) - the object detections
* **"/features_topic"**
  ([vineslam_msgs::FeatureArray](https://gitlab.inesctec.pt/agrob/vineslam_stack/vineslam/-/blob/master/vineslam_msgs/msg/FeatureArray.msg)) - the object detections
* **"/scan_topic"**
  ([sensor_msgs::PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) - the object detections
* **"/odom_topic"**
  ([nav_msgs::Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)) - the robot wheel Odometry
* **"/gps_topic"**
  ([sensor_msgs::NavSatFix](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html)) - the GPS topic (optional)

### Published topics (main)

* **/vineslam/pose**
  ([geometry_msgs::PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)) - the 6-DoF robot pose
* **/vineslam/poses**
  ([geometry_msgs::PoseArray](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseArray.html)) - the 6-DoF particles pose
* **/vineslam/map2D**
  ([visualization_msgs::MarkerArray](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html)) - the 2D semantic map
* **/vineslam/map3D/SURF**
  ([sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) - the 3D SURF-based map
* **/vineslam/map3D/corners**
  ([sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) - the 3D PointCloud corner feature map
* **/vineslam/map3D/planars**
  ([sensor_msgs::PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) - the 3D PointCloud planar features map
* **/vineslam/map3D/planes**
  ([visualization_msgs::MarkerArray](http://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html)) - the 3D PointCloud plane features map
* **/vineslam/elevationMap**
  ([visualization_msgs::MarkerArray](http://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html)) - the ground elevation map


### Transform broadcast

* "/odom" to **world_frame_id** (map).

### Parameters

To tune the system parameters, you can change them in `./vineslam_ros/config/setup_slam.yaml`.

All the parameters are set to their **default values**, so you can run the system witouth changing them.

* **system**:
    * **robot_model** - [agrob/rocha] robot model to use (will have impact in the tf broadcasting).
    * **world_frame_id** - name of the world reference frame.
    * **use_semantic_features** - [True/False] whether to use or not the 2D semantic feature map
    * **use_lidar_features** - [True/False] whether to use or not the 3D LiDAR features
    * **use_image_features** - [True/False] whether to use or not the 3D image features
    * **use_gps** - [True/False] whether to use or not the GPS on 6-DoF Localization
* **camera_info**:
   * **fx** - horizontal focal length (pixels)
    * **cx** - x coordinate of image principal point (pixels)
* **multilayer-mapping**:
    * **grid_map**:
        * **origin**:
            * **x** - x coordinate of grid map origin
            * **y** - y coordinate of grid map origin
            * **z** - z coordinate of grid map origin
        * **width** - map width (meters)
        * **length** - map lenght (meters)
        * **height** - map height (meters)
        * **resolution** - map resolution (meters)
        * **save_map** - [True/False] whether to save or not the map when the system ends
        * **output_file** - name of the map output file (.xml)
        * **input_file** - name of the map input file (.xml) if using localization-only
* **pf**:
    * **n_particles** - number of particles of the particle filter
    * **sigma_xx** - motion model xx constant (meters)
    * **sigma_yy** - motion model yy constant (meters)
    * **sigma_zz** - motion model zz constant (meters)
    * **sigma_RR** - motion model RR constant (radians)
    * **sigma_PP** - motion model PP constant (radians) 
    * **sigma_RR** - motion model YY constant (radians)
    * **k_clusters** - number of clusters to consider at the end of the particle filter