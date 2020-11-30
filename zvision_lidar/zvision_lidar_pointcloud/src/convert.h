/* -*- mode: C++ -*- */
/*
/*
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  Copyright (C) 2019 Zvision
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw ZVISION 3D LIDAR packets to PointCloud2.

*/
#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <pcl/filters/voxel_grid.h>

#include "zvision_lidar_pointcloud/CloudNodeConfig.h"
#include "rawdata.h"
#include "tools/tools.h"

namespace zvision_lidar_pointcloud
{
class Convert
{
public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~Convert()
  {
  }

private:
  void callback(zvision_lidar_pointcloud::CloudNodeConfig& config, uint32_t level);

  void processScan(const zvision_lidar_msgs::zvisionLidarScan::ConstPtr& scanMsg);

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<zvision_lidar_pointcloud::CloudNodeConfig> > srv_;

  zvision::LidarType device_type_;
  boost::shared_ptr<zvision_lidar_rawdata::RawData> data_;
  ros::Subscriber zvision_lidar_scan_;
  ros::Publisher output_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  bool filter_enable_;
  float leaf_size_;

};

}  // namespace zvision_lidar_pointcloud
#endif
