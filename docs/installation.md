# VineSLAM installation guide

--------------------

### General ROS dependencies

```
sudo apt-get install ros-melodic-geographic-msgs ros-melodic-vision-msgs ros-melodic-pcl-ros
```

### Non-catkin dependencies

* ROS melodic

```
mkdir -p ~/vineslam_ws_isolated/src
cd ~/vineslam_ws_isolated/src
git clone git@gitlab.inesctec.pt:agrob/third_party_component/opencv.git -b agro_devel
git clone git@gitlab.inesctec.pt:agrob/third_party_component/opencv_contrib.git -b 3.2.0
cd ~/vineslam_ws_isolated
source /opt/ros/melodic/setup.bash
catkin_make_isolated -DCMAKE_BUILD_TYPE=Release -DENABLE_PRECOMPILED_HEADERS=OFF -DOPENCV_GENERATE_PKGCONFIG=ON -DOPENCV_EXTRA_MODULES_PATH=~/catkin_ws_agro_nav_tests_isolated/src/opencv_contrib/modules
```

* ROS noetic

```
mkdir -p ~/vineslam_ws_isolated/src
cd ~/vineslam_ws_isolated/src
git clone git@gitlab.inesctec.pt:agrob/third_party_component/opencv.git -b agro_noetic_devel
git clone git@gitlab.inesctec.pt:agrob/third_party_component/opencv_contrib.git -b 4.2.0
cd ~/vineslam_ws_isolated
source /opt/ros/melodic/setup.bash
catkin_make_isolated -DCMAKE_BUILD_TYPE=Release -DENABLE_PRECOMPILED_HEADERS=OFF -DOPENCV_GENERATE_PKGCONFIG=ON -DOPENCV_ENABLE_NONFREE=ON -DOPENCV_EXTRA_MODULES_PATH=~/catkin_ws_agro_nav_tests_isolated/src/opencv_contrib/modules
```
### Compile catkin dependencies

```
mkdir -p ~/vineslam_ws/src
cd ~/vineslam_ws/src
catkin_init_workspace
git clone git@gitlab.inesctec.pt:agrob/agrob_map_transform.git -b master
git clone git@gitlab.inesctec.pt:agrob/vineslam.git -b master
cd ~/vineslam_ws
source ~/vineslam_ws_isolated/devel_isolated/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```