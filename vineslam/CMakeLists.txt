cmake_minimum_required(VERSION 3.5)
project(vineslam)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        include
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

ament_export_targets(export_vineslam HAS_LIBRARY_TARGET)

ament_export_dependencies(
        ament_cmake
        Eigen3
)

ament_target_dependencies(vineslam
        ament_cmake
        ${Eigen3_LIBRARIES})

install(TARGETS vineslam
        EXPORT export_vineslam
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        RUNTIME DESTINATION bin)

install(DIRECTORY include/vineslam
        DESTINATION include/vineslam)

ament_package()