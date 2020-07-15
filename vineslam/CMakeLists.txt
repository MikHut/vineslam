cmake_minimum_required(VERSION 2.8.3)
project(vineslam)

add_compile_options(-std=c++11)
add_definitions(-DTEST)

find_package(catkin REQUIRED)

set("OpenCV_DIR" /usr/local/include/opencv2)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7)

catkin_package(
        INCLUDE_DIRS
        ${PROJECT_NAME}/src
        ${PROJECT_NAME}/src/math
        ${PROJECT_NAME}/src/mapXML
        ${PROJECT_NAME}/src/utils
        LIBRARIES vineslam
)

include_directories(
        ${PROJECT_NAME}/src
        ${catkin_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

add_library(vineslam
        ${PROJECT_NAME}/src/mapper2D.cpp
        ${PROJECT_NAME}/src/mapper3D.cpp
        ${PROJECT_NAME}/src/kf.cpp
        ${PROJECT_NAME}/src/localizer.cpp
        ${PROJECT_NAME}/src/pf.cpp
        ${PROJECT_NAME}/src/occupancy_map.cpp
        ${PROJECT_NAME}/src/icp.cpp
        ${PROJECT_NAME}/src/mapXML/map_writer.cpp
        ${PROJECT_NAME}/src/mapXML/map_parser.cpp)

target_link_libraries(vineslam
        ${catkin_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PYTHON_LIBRARIES})

install(TARGETS vineslam
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY ${PROJECT_NAME}/src
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".cpp" EXCLUDE)
