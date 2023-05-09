cmake_minimum_required(VERSION 2.8.3)
project(exercise3)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	exercise2
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf2
	tf2_geometry_msgs
)

find_package(OpenCV)

catkin_package(CATKIN_DEPENDS rospy roscpp)

catkin_install_python(PROGRAMS scripts/goals.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/real_goals.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(map_goals src/map_goals.cpp)
target_link_libraries(map_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(hw3_goals src/hw3_goals.cpp)
target_link_libraries(hw3_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(task1_goals src/task1_goals.cpp)
target_link_libraries(task1_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

