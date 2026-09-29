#!/bin/bash
echo "change compile to catkin"
cp zvlidar_sdk/CMakeLists_ros1.txt zvlidar_sdk/CMakeLists.txt
cp zvlidar_msgs/CMakeLists_ros1.txt zvlidar_msgs/CMakeLists.txt

cp zvlidar_msgs/package_ros1.xml zvlidar_msgs/package.xml
cp zvlidar_msgs/msg/ZvisionLidarPacket_ros1.msg zvlidar_msgs/msg/ZvisionLidarPacket.msg
cp zvlidar_msgs/msg/ZvisionLidarScan_ros1.msg zvlidar_msgs/msg/ZvisionLidarScan.msg

cp zvlidar_sdk/package_ros1.xml zvlidar_sdk/package.xml 
