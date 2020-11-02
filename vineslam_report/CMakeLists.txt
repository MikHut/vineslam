cmake_minimum_required(VERSION 2.8.0)
project(vineslam_report)

set("OpenCV_DIR" /usr/local/include/opencv2)
find_package(OpenCV 3.2.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        qt_build
        roscpp
        vineslam_ros
        vineslam_msgs)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

rosbuild_prepare_qt4(QtCore QtGui)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/vineslam_report/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(vineslam_report ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(vineslam_report ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS vineslam_report RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
