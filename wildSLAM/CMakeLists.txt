cmake_minimum_required(VERSION 2.8.3)
project(wildSLAM)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        vision_msgs pcl_ros)

set("OpenCV_DIR" /usr/local/include/opencv2)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS ${PROJECT_NAME}/src ${PROJECT_NAME}/src/math
        LIBRARIES wildSLAM
)

include_directories(
        ${PROJECT_NAME}/src
        ${catkin_INCLUDE_DIRS}
)

add_library(wildSLAM
        ${PROJECT_NAME}/src/mapper_2d.cpp
        ${PROJECT_NAME}/src/mapper_3d.cpp
        ${PROJECT_NAME}/src/kf.cpp
        ${PROJECT_NAME}/src/localizer.cpp
        ${PROJECT_NAME}/src/pf.cpp
        ${PROJECT_NAME}/src/occupancy_map.cpp
        ${PROJECT_NAME}/src/icp.cpp)

target_link_libraries(wildSLAM
        ${catkin_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${OpenCV_LIBRARIES})

install(TARGETS wildSLAM
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY ${PROJECT_NAME}/src
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".cpp" EXCLUDE)
