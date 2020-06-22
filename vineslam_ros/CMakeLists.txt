cmake_minimum_required(VERSION 2.8.3)
project(vineslam_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        vineslam
        roscpp
        std_msgs
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

SET(${PROJECT_NAME}_SOURCES
        src/main.cpp
        src/visual.cpp
        src/${PROJECT_NAME}.cpp
        src/${PROJECT_NAME}_node.cpp)

add_executable(${PROJECT_NAME}
        ${${PROJECT_NAME}_SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        yaml-cpp)
