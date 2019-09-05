cmake_minimum_required(VERSION 2.8.3)
project(odometer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
                    roscpp
                    std_msgs
                    estimator
                    darknet_ros_msgs)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${estimator_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/odometer.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
