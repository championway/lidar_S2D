# lidar_S2D - LIDAR Point Cloud from Sparse to Dense

# Development Environment
- Ubuntu 16.04
- [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Point Cloud Library](http://pointclouds.org/)
- [Realsense ROS](https://github.com/intel-ros/realsense)
- Gazebo 7.1

# How to run

## Installation
```
$ sudo apt-get install ros-kinetic-gazebo-ros-*
```

## Build
```
$ cd
$ git clone https://github.com/championway/lidar_S2D
$ cd ~/lidar_S2D/catkin_ws
$ source /opt/ros/kinetic/setup.bash
$ catkin_make
```
Note:
Do the following everytime as you open new terminals

```
$ cd ~/lidar_S2D/catkin_ws
$ source devel/setup.bash
```

# Gazebo Equipment

## Sensor

### Depth Camera
- Realsense D435

### LIDAR
- 16-bin 3D LIDAR
