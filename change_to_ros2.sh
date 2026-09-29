#!/bin/bash
echo "change compile to colcon"
cp zvlidar_sdk/CMakeLists_ros2.txt zvlidar_sdk/CMakeLists.txt
cp zvlidar_msgs/CMakeLists_ros2.txt zvlidar_msgs/CMakeLists.txt

cp zvlidar_msgs/package_ros2.xml zvlidar_msgs/package.xml
cp zvlidar_msgs/msg/ZvisionLidarPacket_ros2.msg zvlidar_msgs/msg/ZvisionLidarPacket.msg
cp zvlidar_msgs/msg/ZvisionLidarScan_ros2.msg zvlidar_msgs/msg/ZvisionLidarScan.msg

cp zvlidar_sdk/package_ros2.xml zvlidar_sdk/package.xml 
