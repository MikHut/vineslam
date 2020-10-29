cmake_minimum_required(VERSION 2.8.3)
project(vineslam_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        vineslam
        vineslam_msgs
        agrob_map_transform
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
        message_generation
        cv_bridge
        image_geometry
        image_transport
        pcl_ros)

add_service_files(FILES
        start_map_registration.srv
        stop_map_registration.srv
        stop_gps_heading_estimation.srv)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${vineslam_INCLUDE_DIRS})

add_executable(slam_node src/slam_node.cpp src/vineslam_ros.cpp src/visualization.cpp)
add_dependencies(slam_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_node
        ${catkin_LIBRARIES})

add_executable(replay_node src/replay_node.cpp src/vineslam_ros.cpp src/visualization.cpp)
add_dependencies(replay_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
target_link_libraries(replay_node
        ${catkin_LIBRARIES})

add_executable(localization_node src/localization_node.cpp src/vineslam_ros.cpp src/visualization.cpp)
add_dependencies(localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization_node
        ${catkin_LIBRARIES})
