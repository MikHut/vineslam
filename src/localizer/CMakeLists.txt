cmake_minimum_required(VERSION 2.8.3)
project(localizer)

add_definitions(-DDEBUG)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
		    roscpp
		    edgetpu_cpp 
                    std_msgs
		    sensor_msgs
		    geometry_msgs
		    nav_msgs
		    tf
		    cv_bridge
		    image_transport)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${edgetpu_cpp_INCLUDE_DIRS}
)

include_directories(${CMAKE_SOURCE_DIR}/edgetpu_cpp/dep/libedgetpu)

SET(${PROJECT_NAME}_SOURCES src/main.cpp src/localizer_node.cpp src/particle_filter.cpp)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node benchmark ${catkin_LIBRARIES}
	${OpenCV_LIBRARIES} ${edgetpu_cpp_LIBRARIES})
