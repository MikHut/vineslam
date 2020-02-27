cmake_minimum_required(VERSION 2.8.3)
project(agriSLAM)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
		    roscpp
                    std_msgs)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS ${PROJECT_NAME}/src
  LIBRARIES agriSLAM
)

include_directories(
  ${PROJECT_NAME}/src
  ${catkin_INCLUDE_DIRS}
)

add_library(agriSLAM ${PROJECT_NAME}/src/mapper.cpp
		     ${PROJECT_NAME}/src/kf.cpp
		     ${PROJECT_NAME}/src/localizer.cpp
		     ${PROJECT_NAME}/src/pf.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} 
				      ${Eigen3_LIBRARIES})
						     
install(TARGETS agriSLAM
  	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY ${PROJECT_NAME}/src
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".cpp" EXCLUDE
)
