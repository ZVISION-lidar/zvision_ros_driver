#pragma once

#ifdef ROS_FOUND
    #include <ros/ros.h>
    #include <ros/package.h>
    #include <ros/publisher.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <sensor_msgs/point_cloud2_iterator.h>
    #include <nodelet/nodelet.h>
    #include <pluginlib/class_list_macros.h>
    
    #define NodeHandle_ ros::NodeHandle 
    #define PointCloud2_ sensor_msgs::PointCloud2
    #define PointField_ sensor_msgs::PointField
    #define zvlidar_msgs_ zvlidar_msgs
#endif

#ifdef ROS2_FOUND
    #include <rclcpp/rclcpp.hpp>
    #include <rclcpp/subscription.hpp>
    #include <rcl_interfaces/msg/floating_point_range.hpp>
    #include <rcl_interfaces/msg/parameter_descriptor.hpp>
    #include <rclcpp_components/register_node_macro.hpp>
    #include <sensor_msgs/msg/point_cloud2.hpp>
    #include <sensor_msgs/point_cloud2_iterator.hpp>
    #include <pluginlib/class_list_macros.hpp>

    #define NodeHandle_ rclcpp::Node
    #define PointCloud2_ sensor_msgs::msg::PointCloud2
    #define PointField_ sensor_msgs::msg::PointField
    #define zvlidar_msgs_ zvlidar_msgs::msg
#endif


