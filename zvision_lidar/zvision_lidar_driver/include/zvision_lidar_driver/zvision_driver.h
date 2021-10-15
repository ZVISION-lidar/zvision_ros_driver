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
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
// #include <dynamic_reconfigure/server.h>
// #include "zvision_lidar_driver/zvisionLidarNodeConfig.h"
#include <pcl/point_types.h>
// #include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "input.h"

namespace zvision_lidar_driver
{
class zvisionLidarDriver final : public rclcpp::Node
{
public:
  /**
 * @brief zvisionLidarDriver
 * @param node          raw packet output topic
 * @param private_nh    nodehandler
 */
  zvisionLidarDriver(const rclcpp::NodeOptions & options);

  ~zvisionLidarDriver() override;

  bool poll(void);
  void pollThread();
private:
  /// Callback for dynamic reconfigure
  // void callback(zvision_lidar_driver::zvisionLidarNodeConfig& config, uint32_t level);

  /// Pointer to dynamic reconfigure service srv_
  // boost::shared_ptr<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double time_offset;    ///< time in seconds added to each  time stamp
  } config_;

  boost::shared_ptr<Input> input_;
  rclcpp::Publisher<zvision_lidar_msgs::msg::ZvisionLidarScan>::SharedPtr output_;
  // Converter convtor_;
  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;//diagnose_topic

  // We use this future/promise pair to notify threads that we are shutting down
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

  // The thread that deals with data
  std::thread poll_thread_;
};

}  // namespace zvision_lidar_driver

#endif
