cmake_minimum_required(VERSION 2.8.3)
project(vineslam_ros)

if($ENV{ROS_DISTRO} STREQUAL "noetic")
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(catkin REQUIRED
        vineslam
        vineslam_msgs
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        interactive_markers
        visualization_msgs
        vision_msgs
        tf
        geometry_msgs
        message_generation
        image_geometry
        pcl_ros)


generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${vineslam_INCLUDE_DIRS})

add_executable(localization_node src/localization_node.cpp src/vineslam_ros.cpp src/visualization.cpp)
add_dependencies(localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization_node
        ${catkin_LIBRARIES})
