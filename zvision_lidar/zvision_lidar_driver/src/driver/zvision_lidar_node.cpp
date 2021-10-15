/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  Copyright (C) 2019 Zvision Technology, Pengfei Cui
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the ZVISION 3D LIDARs.
 */
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include "zvision_driver.h"
//#include "std_msgs/String.h"
#include "std_msgs/msg/string.hpp"

using namespace zvision_lidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

int main(int argc, char** argv)
{
//  ros::init(argc, argv, "zvision_driver");
//  ros::NodeHandle node;
//  ros::NodeHandle private_nh("~");
//
//  signal(SIGINT, my_handler);
//
//  // start the driver
//  zvision_lidar_driver::zvisionLidarDriver dvr(node, private_nh);
//  // loop until shut down or end of file
//  while (ros::ok() && dvr.poll())
//  {
//    ros::spinOnce();
//  }
    // Force flush of the stdout buffer.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::spin(
            std::make_shared<zvision_lidar_driver::zvisionLidarDriver>(
                    rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
  return 0;
}
