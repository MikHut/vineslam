cmake_minimum_required(VERSION 2.8.3)
project(mapper)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
		    qt_build
                    roscpp
                    std_msgs
		    sensor_msgs
		    geometry_msgs
		    nav_msgs
		    tf
		    cv_bridge
		    image_transport
                    darknet_ros_msgs)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# to find main_window_ui.h
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_HEADERS include/mapper/mapper.hpp
	include/mapper/landmark_processor.hpp utils/utils.hpp utils/qnode.hpp
	include/mapper/main_window.hpp include/mapper/mapper_node.hpp)
SET(${PROJECT_NAME}_SOURCES src/main_window.cpp src/mapper_node.cpp src/mapper.cpp
	src/landmark_processor.cpp utils/qnode.cpp src/main.cpp)
SET(${PROJECT_NAME}_FORMS   ui/main_window.ui)
QT5_WRAP_CPP(${PROJECT_NAME}_HEADERS_MOC ${${PROJECT_NAME}_HEADERS})
QT5_WRAP_UI(${PROJECT_NAME}_FORMS_HEADERS ${${PROJECT_NAME}_FORMS})

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS_MOC}
	${${PROJECT_NAME}_FORMS_HEADERS})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${QT_LIBRARIES})

qt5_use_modules(${PROJECT_NAME}_node Core Gui Widgets)
