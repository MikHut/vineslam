cmake_minimum_required(VERSION 2.8.3)
project(agrob_map_transform)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  eigen_conversions
  geographic_msgs
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  xmlrpcpp
)

find_package (OpenCV)


find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp)

find_package(PCL REQUIRED)

# Attempt to find Eigen using its own CMake module.
# If that fails, fall back to cmake_modules package.
find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++14;-Wall")
add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
roslint_cpp()

###################################
## catkin specific configuration ##
###################################
add_service_files(
  FILES
    GetState.srv
    SetDatum.srv
    SetPose.srv
    ToggleFilterProcessing.srv
    GetPolars.srv
    GetPose.srv
)

generate_messages(
  DEPENDENCIES
    geographic_msgs
    geometry_msgs
    std_msgs
    sensor_msgs
)

catkin_package(
     INCLUDE_DIRS
        include
     LIBRARIES
        agrob_gnss_transform
     CATKIN_DEPENDS
        cmake_modules
        diagnostic_msgs
        diagnostic_updater
        eigen_conversions
        geographic_msgs
        geometry_msgs
        message_filters
        message_runtime
        nav_msgs
        roscpp
        sensor_msgs
        std_msgs
        std_srvs
        tf2
        tf2_geometry_msgs
        tf2_ros
        xmlrpcpp
     DEPENDS
        ${EIGEN_PACKAGE}
        YAML_CPP PCL
        sensor_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${YAML_CPP_LIBRARY_DIRS})

# Library definitions
add_library(agrob_gnss_transform src/agrob_gnss_transform.cpp)
add_dependencies(agrob_gnss_transform ${PROJECT_NAME}_gencpp)

# Executables
add_executable(agrob_gnss_transform_node src/agrob_gnss_transform_node.cpp)
add_dependencies(agrob_gnss_transform_node ${PROJECT_NAME}_gencpp)

# Linking
target_link_libraries(agrob_gnss_transform  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES}   ${OpenCV_LIBRARIES})
target_link_libraries(agrob_gnss_transform_node agrob_gnss_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES}    ${OpenCV_LIBRARIES}
        )




