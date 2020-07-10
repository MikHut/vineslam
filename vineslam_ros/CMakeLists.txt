cmake_minimum_required(VERSION 2.8.3)
project(vineslam_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        vineslam
        agrob_map_transform
        roscpp
        std_msgs
        genmsg
        sensor_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        vision_msgs
        tf
        geometry_msgs
        message_filters
        cv_bridge
        image_geometry
        image_transport
        pcl_ros)

find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${vineslam_INCLUDE_DIRS})

add_executable(slam_node src/slam_node.cpp)
add_dependencies(slam_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_node
        ${catkin_LIBRARIES}
        yaml-cpp)

add_executable(localization_node src/localization_node.cpp)
add_dependencies(localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization_node
        ${catkin_LIBRARIES}
        yaml-cpp)
