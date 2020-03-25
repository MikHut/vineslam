cmake_minimum_required(VERSION 2.8.3)
project(detector)

add_definitions(-DDEBUG)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
		    wildSLAM
		    edgetpu_cpp 
		    agrob_map_transform
		    roscpp
                    std_msgs
		    vision_msgs
		    sensor_msgs
		    geometry_msgs
		    visualization_msgs
		    nav_msgs
		    tf
		    geometry_msgs
		    message_filters
		    cv_bridge
		    image_geometry
		    image_transport)

find_package(OpenCV REQUIRED)

catkin_package()

message(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/../edgetpu_cpp/dep/libedgetpu)
include_directories(benchmark /usr/local/include/benchmark)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${wildSLAM_INCLUDE_DIRS}
  ${edgetpu_cpp_INCLUDE_DIRS}
  ${agrob_map_transform_cpp_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_SOURCES src/main.cpp src/${PROJECT_NAME}_node.cpp src/debug.cpp)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node benchmark ${catkin_LIBRARIES} 
					             ${OpenCV_LIBRARIES}
						     ${edgetpu_cpp_LIBRARIES})
