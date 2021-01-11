cmake_minimum_required(VERSION 2.8.3)
project(vineslam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)

find_package(OpenCV 4.2.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES vineslam
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

add_library(vineslam
        src/mapping/landmark_mapping.cpp
        src/mapping/landmark_ekf.cpp
        src/mapping/visual_mapping.cpp
        src/mapping/lidar_mapping.cpp
        src/mapping/occupancy_map.cpp
        src/mapping/elevation_map.cpp
        src/localization/localizer.cpp
        src/localization/pf.cpp
        src/mapxml/map_writer.cpp
        src/mapxml/map_parser.cpp)

target_link_libraries(vineslam
        ${catkin_LIBRARIES}
        ${Eigen3_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PYTHON_LIBRARIES})

install(TARGETS vineslam
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/vineslam
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".cpp" EXCLUDE)
