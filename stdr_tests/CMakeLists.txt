cmake_minimum_required(VERSION 2.8.3)
project(stdr_tests)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	tf
	sensor_msgs
	nav_msgs
	stdr_msgs
	stdr_robot
	stdr_server
	stdr_gui
)

include_directories( include
	${catkin_INCLUDE_DIRS}
)

include_directories(
	${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
    CATKIN_DEPENDS # TODO
		roscpp
		roslib
		tf
		stdr_msgs
		stdr_robot
		stdr_gui
		stdr_server
		sensor_msgs
		nav_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

#----------------------------------------------------------------------------------------------------#

set(CMAKE_BUILD_TYPE Debug)

add_executable(stdr_gui_range
	src/test.cpp
)
add_dependencies(stdr_gui_range stdr_msgs_gencpp stdr_robot_gencpp stdr_server_gencpp stdr_gui_gencpp)	# Wait for std messages to build

target_link_libraries(stdr_gui_range 
	${catkin_LIBRARIES} gtest gmock
)
