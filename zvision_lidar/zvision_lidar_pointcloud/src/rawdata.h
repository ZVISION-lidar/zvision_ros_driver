/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  Copyright (C) 2019 Zvision
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Robosense 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Tony Zhang
 *  @author Pengfei Cui
 */

#ifndef _RAWDATA_H
#define _RAWDATA_H

#include <ros/ros.h>
#include <ros/package.h>
#include <zvision_lidar_msgs/zvisionLidarPacket.h>
#include <zvision_lidar_msgs/zvisionLidarScan.h>
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <iostream>
namespace zvision_lidar_rawdata
{

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
static const int PACKET_SIZE_ML30 = 1304;//PACKET_SIZE
static const int PACKET_SIZE_ML30S_A1 = 1346;
static const int GROUP_PER_PACKET_ML30 = 80;
static const int GROUP_PER_PACKET = 40;
static const int POINT_PER_GROUP_ML30 = 3;
static const int POINT_PER_GROUP = 8;

/** \brief ZVISION LIDAR data conversion class */
class RawData
{
public:
  enum LIDAR_TYPE{
  	ML30,
  	ML30S_A1
  };

  RawData();

  ~RawData();

  /*load the cablibrated files: angle*/
  void loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh);

  /*unpack the ML30S-A1 UDP packet and opuput PCL PointXYZI type*/
  void unpack(const zvision_lidar_msgs::zvisionLidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);

  /*gps time stamp*/
  void getTimeStampFromUdpPkt(const zvision_lidar_msgs::zvisionLidarPacket& pkt, long long int& unix_sec, long long int&  unix_microsec);

  bool is_init_angle_;
  int timestamp_type;/*0:local time, 1:get the timestamp from udp*/


  LIDAR_TYPE type;
  double *pdElevationAngle;
  double *pdAzimuthAngle;
  double x_trans;
  double y_trans;
  double z_trans;
  double x_rotation;
  double y_rotation;
  double z_rotation;

};

}  // namespace zvision_lidar_rawdata

#endif  // __RAWDATA_H
