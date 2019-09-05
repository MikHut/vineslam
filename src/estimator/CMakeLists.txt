cmake_minimum_required(VERSION 2.8.3)
project(estimator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(  
  INCLUDE_DIRS include
  LIBRARIES estimator
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(estimator
  src/landmark_processor.cpp)


install(TARGETS estimator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
