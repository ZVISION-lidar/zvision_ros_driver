/**
 * @file pointcloud_status.cc
 * @author dong.wang (dong.wang@zvision.xyz)
 * @brief 
 * @version 0.1
 * @date 2023-04-28
 * 
 * @copyright Copyright (C) 2023 Zvision Dong Wang
 * 
 */

#include "pointcloud_status.h"

namespace zvision_lidar_pointcloud
{

PointCloudStatus::PointCloudStatus(
    int statistic_frame_num,
    int roi_sample_lines, 
    int roi_interval,
    int min_roi_pointnum,
    double z_height,
    double roi_z_diff_threshold,
    double error_rate_threshold):statistic_frame_num_(statistic_frame_num),roi_sample_lines_(roi_sample_lines),
      roi_interval_(roi_interval),min_roi_pointnum_(min_roi_pointnum),z_height_(z_height),roi_z_diff_threshold_(roi_z_diff_threshold),
      error_rate_threshold_(error_rate_threshold){

}

PointCloudStatus::~PointCloudStatus(){

}

bool PointCloudStatus::is_zero(double x) const{
    return std::abs(x) < std::numeric_limits<double>::epsilon();
}

void PointCloudStatus::set_threshold(
  int& statistic_frame_num,
  int& roi_sample_lines,
  int& roi_interval,
  int& min_roi_pointnum,
  double& z_height,
  double& roi_z_diff_threshold,
  double& error_rate_threshold){

  statistic_frame_num_ = statistic_frame_num;
  roi_sample_lines_ = roi_sample_lines;
  roi_interval_ = roi_interval;
  min_roi_pointnum_ = min_roi_pointnum;
  z_height_ = z_height;
  roi_z_diff_threshold_ = roi_z_diff_threshold;
  error_rate_threshold_ = error_rate_threshold;
}

PointCloudStatus::LidarStatus PointCloudStatus::check_status(
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){
  
  // std::vector<int> roi_indexes;
  bool is_roi_point = false;
  int valid_pointnum = 0;
  double sum_z = 0.0, mean_z = 0.0;

  for(size_t i = 0; i < input_cloud->size(); i++){
    // [0,6,1,7,2,4,3,5]
    // 1, 2 两个视场
    if(i % 8 == 2){
      // 1 视场
      if(i % (8 * roi_interval_) == 2 && i <= (8 * 80* roi_sample_lines_)) {
        // roi_indexes.push_back(i);
        is_roi_point = true;
      }
    }else if(i % 8 == 4){
      // 2 视场
      if(i % (8 * roi_interval_) == 4 && i <= (8 * 80* roi_sample_lines_)) {
        // roi_indexes.push_back(i);
        is_roi_point = true;
      }
    }
    if(is_roi_point){
      
      pcl::PointXYZI point = input_cloud->at(i);
      if(pcl::isFinite(point) && (!is_zero(point.x)||!is_zero(point.y)||!is_zero(point.z))){
        valid_pointnum+=1;
        sum_z += point.z;
      }
      is_roi_point = false;
    }
  }

  if(valid_pointnum == 0){
    error_num_ += 1;
    ROS_DEBUG("valid_pointnum = %d", valid_pointnum);
  }else{

    mean_z = sum_z / double(valid_pointnum);

    double mean_z_error = fabs(mean_z - z_height_);
    if(mean_z_error < roi_z_diff_threshold_ && valid_pointnum >= min_roi_pointnum_){

    }else{
      error_num_ += 1;
      ROS_DEBUG("valid_pointnum = %d", valid_pointnum);
      ROS_DEBUG("mean_z_error = %f", mean_z_error);
    }
  }

  current_frame_count_ += 1;
  if(current_frame_count_ > statistic_frame_num_) {
    double error_rate = (double)error_num_ / (double)current_frame_count_;
    if(error_rate > error_rate_threshold_) {
      // error
      current_frame_count_ = 0;
      error_num_ = 0;
      status_ = LidarStatus::Error;
      ROS_DEBUG("error_rate = %f", error_rate);
    } else{
      // normal
      // clean
      current_frame_count_ = 0;
      error_num_ = 0;
      status_ = LidarStatus::Normal;
    }
    ROS_DEBUG("status = %d", status_);
  }
  return status_;
}
}