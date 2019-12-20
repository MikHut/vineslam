cmake_minimum_required(VERSION 2.8.3)
project(mapper)

add_definitions(-DDEBUG)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
		    edgetpu_cpp 
		    roscpp
                    std_msgs
		    sensor_msgs
		    geometry_msgs
		    visualization_msgs
		    nav_msgs
		    tf
		    geometry_msgs
		    cv_bridge
		    image_geometry
		    image_transport)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${CMAKE_SOURCE_DIR}/edgetpu_cpp/dep/libedgetpu)
include_directories(benchmark /usr/local/include/benchmark)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${edgetpu_cpp_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_SOURCES src/mapper_node.cpp src/mapper.cpp
	src/landmark_processor.cpp src/debug.cpp src/main.cpp src/kf.cpp)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node benchmark ${catkin_LIBRARIES} 
					             ${OpenCV_LIBRARIES}
						     ${Eigen3_LIBRARIES}
						     ${edgetpu_cpp_LIBRARIES})
