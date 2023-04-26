cmake_minimum_required(VERSION 2.8.3)
project(exercise7)

## Find system dependencies
## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy
)


## Declare the catkin package
catkin_package(CATKIN_DEPENDS rospy roscpp)

catkin_install_python(PROGRAMS scripts/detect_parking.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
