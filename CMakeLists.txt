cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(hawk_teleoperations)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg mavros_msgs)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hawk_teleoperations src/main.cpp)
target_link_libraries(hawk_teleoperations ${catkin_LIBRARIES})
add_dependencies(hawk_teleoperations hawk_teleoperations_generate_messages_cpp)


# second node
add_executable(hawk_gps src/gps_nav.cpp)
target_link_libraries(hawk_gps ${catkin_LIBRARIES})
add_dependencies(hawk_gps hawk_gps_generate_messages_cpp)


install(
	TARGETS hawk_teleoperations
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
