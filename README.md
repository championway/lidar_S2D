# lidar_S2D - LIDAR Point Cloud from Sparse to Dense using GAN

![32 Bin LIDAR Point Cloud From Sparse To Dense](https://github.com/championway/lidar_S2D/blob/master/image/simple_demo.gif)

# Development Environment
- Ubuntu 16.04
- [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Point Cloud Library](http://pointclouds.org/)
- [Realsense ROS](https://github.com/intel-ros/realsense)
- Gazebo 7.14

# Gazebo Equipment

## Sensor

### Depth Camera
- Kinect depth camera

### LIDAR
- VLP-16 LIDAR
- VLP-32 LIDAR

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
$ cd ~/lidar_S2D
$ source environment.sh
```

## Open gazebo
```
$ roslaunch gazebo_run test.launch
```

## Colllect LIDAR depth image 
```
$ rosrun rgbd_camera image_process
```

## LIDAR point cloud to depth image 
```
$ rosrun rgbd_camera lidar2depth
```

## Depth image to LIDAR point cloud
```
$ rosrun rgbd_camera depth2lidar
```