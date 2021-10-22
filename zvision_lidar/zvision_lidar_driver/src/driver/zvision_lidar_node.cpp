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
#include <rclcpp/rclcpp.hpp>
#include "zvision_driver.h"
#include "std_msgs/msg/string.hpp"

using namespace zvision_lidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

int main(int argc, char** argv)
{
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
