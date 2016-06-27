cmake_minimum_required(VERSION 2.8.3)
project(bag_to_depth)

set(catkin_RUN_PACKAGES
  roscpp
  sensor_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  rosbag
)

include_directories(
	include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp std_msgs rosbag

)

add_executable(bagtodepth
  src/bag_to_depth_main.cpp
  src/bag_to_depth.cpp
)
target_link_libraries(bagtodepth
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)













