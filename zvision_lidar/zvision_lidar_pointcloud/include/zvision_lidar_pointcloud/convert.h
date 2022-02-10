/* -*- mode: C++ -*- */
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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/voxel_grid.h>
#include "rawdata.h"
#include "tools.h"

namespace zvision_lidar_pointcloud
{
class Convert final : public rclcpp::Node
{
public:
  explicit Convert(const rclcpp::NodeOptions & options);

  ~Convert() override
  {
  }

  enum DownsampleType
  {
      None = 0,
      Voxel = 1,
      Line = 2,
  };

private:

  void processScan(const zvision_lidar_msgs::msg::ZvisionLidarScan::SharedPtr scanMsg);
  int* get_nearest_point_index();
  double calDistance(pcl::PointXYZI a, pcl::PointXYZI b);
  zvision::LidarType device_type_;
  boost::shared_ptr<zvision_lidar_rawdata::RawData> data_;
  rclcpp::Subscription<zvision_lidar_msgs::msg::ZvisionLidarScan>::SharedPtr zvision_lidar_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  bool filter_enable_;
  bool use_outlier_removal;
  double outlier_th;
  float leaf_size_;
  int line_sample_;
  DownsampleType downsample_type_;
  int* table_;
};

}  // namespace zvision_lidar_pointcloud
#endif
