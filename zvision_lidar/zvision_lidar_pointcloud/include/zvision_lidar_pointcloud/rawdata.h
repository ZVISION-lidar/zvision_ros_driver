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

//#include <ros/ros.h>
//#include <ros/package.h>
#include <rclcpp/rclcpp.hpp>
#include <zvision_lidar_msgs/msg/zvision_lidar_packet.hpp>
#include <zvision_lidar_msgs/msg/zvision_lidar_scan.hpp>
#include "std_msgs/msg/string.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// #include <pcl_ros/impl/transforms.hpp>
// #include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <iostream>
#include <memory>
#include <thread>
#include "tools.h"


namespace zvision
{
    class CalibrationData;
    class PointCalibrationTable;
    enum LidarType;
}

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
static const int GROUP_PER_PACKET_MLX = 80;
static const int GROUP_PER_PACKET_MLXS = 80;
static const int GROUP_PER_PACKET = 40;
static const int POINT_PER_GROUP_ML30 = 3;
static const int POINT_PER_GROUP = 8;
static const int POINT_PER_GROUP_MLX = 3;
static const int POINT_PER_GROUP_MLXS = 3;
static const int MAX_POINTS = 204800;

/** \brief ZVISION LIDAR data conversion class */
class RawData final : public rclcpp::Node
{
public:

  RawData(const rclcpp::NodeOptions & options);

  ~RawData();

  /*load the cablibrated files: angle*/
  void loadConfigFile();

  /*calibration data is init ok or not*/
  bool isCalibrationInitOk();

  /*unpack the ML30S-A1 UDP packet and opuput PCL PointXYZI type*/
  void unpack(const zvision_lidar_msgs::msg::ZvisionLidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);

  /*gps time stamp*/
  void getTimeStampFromUdpPkt(const zvision_lidar_msgs::msg::ZvisionLidarPacket& pkt, long long int& unix_sec, long long int&  unix_microsec);

  bool cal_init_ok_;
  bool use_lidar_time_;/*0:local time, 1:get the timestamp from udp*/
  std::string dev_ip_;

  zvision::LidarType device_type_;
  std::shared_ptr<zvision::CalibrationData> cal_;
  std::shared_ptr<zvision::PointCalibrationTable> cal_lut_;
  std::vector<int> point_line_number_;

  std::shared_ptr<std::thread> online_calibration_thread_ = NULL;

  double x_trans;
  double y_trans;
  double z_trans;
  double x_rotation;
  double y_rotation;
  double z_rotation;

protected:
  void PollCalibrationData(void);

};

}  // namespace zvision_lidar_rawdata

#endif  // __RAWDATA_H
