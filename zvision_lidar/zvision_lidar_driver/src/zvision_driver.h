/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 * 	Copyright (C) 2019, Zvision, Pengfei Cui
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the ZVISION LIDAR 3D LIDARs
 */
#ifndef _ZVISION_DRIVER_H_
#define _ZVISION_DRIVER_H_

#include <string>
#include<functional>
#include <ros/ros.h>
#include <ros/package.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include "zvision_lidar_driver/zvisionLidarNodeConfig.h"
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "input.h"

namespace zvision_lidar_driver
{
class zvisionLidarDriver
{
public:
  /**
 * @brief zvisionLidarDriver
 * @param node          raw packet output topic
 * @param private_nh    nodehandler
 */
  zvisionLidarDriver(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~zvisionLidarDriver()
  {
  }

  bool poll(void);

private:
  /// Callback for dynamic reconfigure
  void callback(zvision_lidar_driver::zvisionLidarNodeConfig& config, uint32_t level);

  /// Pointer to dynamic reconfigure service srv_
  /**/std::shared_ptr<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> > srv_;
  bool pub_cfg_srv_;
  // configuration parameters
  struct
  {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double time_offset;    ///< time in seconds added to each  time stamp
  } config_;

  /**/std::shared_ptr<Input> input_;
  ros::Publisher output_;
  // Converter convtor_;
  /** diagnostics updater */
  std::shared_ptr<diagnostic_updater::Updater> diagnostics_ = nullptr;
  //diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  /**/std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_ = nullptr;//diagnose_topic
};

}  // namespace zvision_lidar_driver

#endif
