# Install wildSLAM and test package

## Dependencies

### Standalone wildSLAM package

* **ROS** - [melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) is the recomended version
* **octomap** - `sudo apt install ros-melodic-octomap*`
* **vision_msgs** - `sudo apt install ros-melodic-vision-msgs`

### Test package

* **google_benchmarks**: follow the cmake installation procedure described here, andd install it at the system using `sudo make install`; clone the repo in test/edgetpu_cpp/dep.
* **flatbuffers**: use cmake to install the repo, and install it at the system using `sudo
  make install`; clone it at test/edgetpu_cpp/dep.
* **glog**: follow the cmake installation procedure described here, and install it at the system using `sudo make install`. clone it at test/edgetpu_cpp/dep.
* **PythonLibs**: install it with sudo apt install python-dev
* **Google tests**:
```
sudo apt-get install cmake libgtest-dev
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
 
# copy or symlink libgtest.a and libgtest_main.a to your /usr/lib folder
sudo cp *.a /usr/lib
```

* Generate Tensorflow-Lite binaries:
```
.test/edgetpu_cpp/dep/libedgetpu/tensorflow/lite/tools/make/build_lib.sh
```

## Installation

To install simply clone and compile the project.
```
cd src/<user>/catkin_ws
git clone https://gitlab.inesctec.pt/agrob/agrobvslam
catkin_make -DCMAKE_BUILD_TYPE:=Release
```
