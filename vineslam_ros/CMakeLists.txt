cmake_minimum_required(VERSION 3.5)
project(vineslam_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vineslam REQUIRED)
find_package(vineslam_msgs REQUIRED)
#find_package(agrob_map_transform REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/StartMapRegistration.srv"
        "srv/StopMapRegistration.srv"
        "srv/SaveMap.srv"
        "srv/StopGpsHeadingEstimation.srv"
        DEPENDENCIES std_msgs)

include_directories(
        include
        ${vineslam_INCLUDE_DIRS})

set(dependencies
        "ament_index_cpp"
        "vineslam"
        "vineslam_msgs"
        "rclcpp"
        "std_msgs"
        "sensor_msgs"
        "nav_msgs"
        "visualization_msgs"
        "vision_msgs"
        "tf2"
        "tf2_ros"
        "cv_bridge"
        "image_geometry"
        "image_transport"
        "pcl_conversions"
        "geometry_msgs")

add_executable(slam_node src/slam_node.cpp src/vineslam_ros.cpp)# src/visualization.cpp)
ament_target_dependencies(slam_node ${dependencies})
rosidl_target_interfaces(slam_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(slam_node ${PCL_LIBRARIES})

add_executable(localization_node src/localization_node.cpp)# src/vineslam_ros.cpp src/visualization.cpp)
ament_target_dependencies(localization_node ${dependencies})
rosidl_target_interfaces(localization_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(localization_node
#        ${catkin_LIBRARIES})

add_executable(mapping_node src/mapping_node.cpp)# src/vineslam_ros.cpp src/visualization.cpp)
ament_target_dependencies(mapping_node ${dependencies})
rosidl_target_interfaces(mapping_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(mapping_node
#        ${catkin_LIBRARIES})

install(TARGETS
        slam_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        config
        launch
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()