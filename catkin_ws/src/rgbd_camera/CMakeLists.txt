cmake_minimum_required(VERSION 2.8.3)
project(rgbd_camera)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  s2d_msgs
  ddynamic_reconfigure
)

catkin_package(
)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_process src/image_process.cpp)
target_link_libraries(image_process ${catkin_LIBRARIES})

add_executable(depth2lidar src/depth2lidar.cpp)
target_link_libraries(depth2lidar ${catkin_LIBRARIES})

add_executable(lidar2depth src/lidar2depth.cpp)
target_link_libraries(lidar2depth ${catkin_LIBRARIES})