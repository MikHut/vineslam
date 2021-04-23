cmake_minimum_required(VERSION 3.5)
project(vineslam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(vineslam
        src/mapping/landmark_mapping.cpp
        src/mapping/landmark_ekf.cpp
        src/mapping/visual_mapping.cpp
        src/mapping/lidar_mapping.cpp
        src/mapping/occupancy_map.cpp
        src/mapping/elevation_map.cpp
        src/localization/localizer.cpp
        src/localization/pf.cpp
        src/map_io/map_writer.cpp
        src/map_io/map_parser.cpp
        src/map_io/elevation_map_writer.cpp
        src/map_io/elevation_map_parser.cpp
        include/vineslam/extern/thread_pool.cpp)

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
        DESTINATION include/)

ament_package()