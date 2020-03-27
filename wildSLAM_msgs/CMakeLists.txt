cmake_minimum_required(VERSION 3.0.2)
project(wildSLAM_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation 
					std_msgs
					nav_msgs
					visualization_msgs)

add_message_files(
  DIRECTORY msg
  FILES feature.msg featureMap.msg
)

generate_messages(DEPENDENCIES std_msgs nav_msgs visualization_msgs)

catkin_package(DEPENDS message_runtime std_msgs nav_msgs visualization_msgs)
