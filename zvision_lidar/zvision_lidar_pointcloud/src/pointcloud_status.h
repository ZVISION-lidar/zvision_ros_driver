/**
 * @file pointcloud_status.h
 * @author dong.wang (dong.wang@zvision.xyz)
 * @brief estimate lidar status by pointcloud
 * @version 0.1
 * @date 2023-04-28
 * 
 * @copyright Copyright (C) 2023 Zvision Dong Wang
 * 
 */

#ifndef ZVISION_PERCEPTION_POINTCLOUD_STATUS_H_
#define ZVISION_PERCEPTION_POINTCLOUD_STATUS_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>

namespace zvision_lidar_pointcloud
{
class PointCloudStatus
{
public: 
  PointCloudStatus(
    int statistic_frame_num,
    int roi_sample_lines, 
    int roi_interval,
    int min_roi_pointnum,
    double z_height,
    double roi_z_diff_threshold,
    double error_rate_threshold
    );
  ~PointCloudStatus();
  enum LidarStatus
  {
    Normal = 0,
    Error = 1
  };
  LidarStatus check_status(
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr);
  void set_threshold(
    int& statistic_frame_num,
    int& roi_sample_lines, 
    int& roi_interval,
    int& min_roi_pointnum,
    double& z_height,
    double& roi_z_diff_threshold,
    double& error_rate_threshold);

private:
  
  int roi_sample_lines_ = 10;
  int roi_interval_ = 10;
  int min_roi_pointnum_ = 60;
  double z_height_ = -0.58;
  int statistic_frame_num_ = 100;
  double roi_z_diff_threshold_ = 0.1;
  double error_rate_threshold_ = 0.5;

  int error_num_ = 0;
  int current_frame_count_ = 0;

  LidarStatus status_ = LidarStatus::Normal;
  bool is_zero(double x) const;
};
}

#endif // ZVISION_PERCEPTION_POINTCLOUD_STATUS_H_
