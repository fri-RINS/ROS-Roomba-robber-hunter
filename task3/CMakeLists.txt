cmake_minimum_required(VERSION 2.8.3)
project(task3)

## Find system dependencies
## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  pyocr
)

add_message_files(DIRECTORY msg
  FILES
  PosterMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)


## Declare the catkin package
catkin_package(CATKIN_DEPENDS rospy roscpp std_msgs
  sensor_msgs
  geometry_msgs
  message_runtime pyocr)
  
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/face_and_poster_detector.py scripts/poster_detector.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


