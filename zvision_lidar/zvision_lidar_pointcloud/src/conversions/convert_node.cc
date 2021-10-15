/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 Robosense, Tony Zhang
 *  Copyright (C) 2019 Zvision, Pengfei Cui
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw ZVISION LIDAR packets to PointCloud2.

*/
#include "convert.h"

/** Main node entry point. */
int main(int argc, char** argv)
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    // handle callbacks until shut down
    rclcpp::spin(
            std::make_shared<zvision_lidar_pointcloud::Convert>(
                    rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
//  ros::init(argc, argv, "cloud_node");
//  ros::NodeHandle node;
//  ros::NodeHandle priv_nh("~");
//
//  // create conversion class, which subscribes to raw data
//  zvision_lidar_pointcloud::Convert conv(node, priv_nh);
//
//  // handle callbacks until shut down
//  ros::spin();
//  return 0;
}
