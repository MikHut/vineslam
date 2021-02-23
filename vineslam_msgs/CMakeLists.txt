cmake_minimum_required(VERSION 3.5)
project(vineslam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Particle.msg"
        "msg/Report.msg"
        "msg/Feature.msg"
        "msg/FeatureArray.msg"
        DEPENDENCIES std_msgs nav_msgs geometry_msgs sensor_msgs visualization_msgs)

ament_package()