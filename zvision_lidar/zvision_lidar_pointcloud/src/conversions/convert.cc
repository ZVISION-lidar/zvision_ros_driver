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
#include "convert.h"
#include "tools.h"
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>
namespace zvision_lidar_pointcloud
{
std::string model;

/** @brief Constructor. */
Convert::Convert(const rclcpp::NodeOptions & options)
:rclcpp::Node("zvision_convert_node", options),
data_(new zvision_lidar_rawdata::RawData(options))
  ,filter_enable_(false)
{

  model = this->declare_parameter("model",std::string("ML30SA1"));
  device_type_ = zvision::LidarTools::GetDeviceTypeFromTypeString(model);

  std::string downsample_string = "";
  downsample_string = this->declare_parameter("downsample_type",std::string(""));
  if(downsample_string == "downsample_line")
  {
      line_sample_ = this->declare_parameter("line_sample",2);
      this->downsample_type_ = DownsampleType::Line;
      RCLCPP_INFO(this->get_logger(),"Downsample type is [Line], 1 in %d.", line_sample_);
  }
  else if(downsample_string == "downsample_voxel")
  {
      leaf_size_ = this->declare_parameter("voxel_leaf_size",0.2f);
      voxel_grid_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      this->downsample_type_ = DownsampleType::Voxel;
      RCLCPP_INFO(this->get_logger(),"Downsample type is [Voxel], leaf size is [%.3f].", leaf_size_);
  }
  else
  {
      this->downsample_type_ = DownsampleType::None;
      RCLCPP_INFO(this->get_logger(),"Downsample type is [None] publish raw pointcloud.");
  }

  output_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("zvision_lidar_points", 20);
  //srv_question
  // srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_pointcloud::CloudNodeConfig> >(private_nh);
  // dynamic_reconfigure::Server<zvision_lidar_pointcloud::CloudNodeConfig>::CallbackType f;

  // f = boost::bind(&Convert::callback, this, _1, _2);
  // srv_->setCallback(f);

  data_->loadConfigFile();

  zvision_lidar_scan_ =
          this->create_subscription<zvision_lidar_msgs::msg::ZvisionLidarScan>(
                  "zvision_lidar_packets", rclcpp::QoS(20),
                  std::bind(&Convert::processScan, this, std::placeholders::_1));

}

// void Convert::callback(zvision_lidar_pointcloud::CloudNodeConfig& config, uint32_t level)
// {
//   RCLCPP_INFO(this->get_logger(),"Reconfigure Request");
//   printf("parameter:%f %f %f %f %f %f\n", config.x_trans, config.y_trans,config.z_trans,config.z_rotation,config.y_rotation,config.z_rotation);
//   data_->x_trans = config.x_trans;
//   data_->y_trans = config.y_trans;
//   data_->z_trans = config.z_trans;
//   data_->x_rotation = config.x_rotation / 180.0 * M_PI;
//   data_->y_rotation = config.y_rotation / 180.0 * M_PI;
//   data_->z_rotation = config.z_rotation / 180.0 * M_PI;
//   // config_.time_offset = config.time_offset;
// }

/** @brief Callback for raw scan messages. *///IMPORTANT
void Convert::processScan(const zvision_lidar_msgs::msg::ZvisionLidarScan::SharedPtr scanMsg)
{
  long long time_from_pkt_s = 0;
  long long time_from_pkt_us = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();

  if(!data_->isCalibrationInitOk())
  {
      RCLCPP_WARN_ONCE(this->get_logger(),"Calibration data is not initialized.");
      return;
  }
  else
      RCLCPP_INFO_ONCE(this->get_logger(),"Calibration data init ok.");

  if(device_type_ == zvision::ML30SA1){
	  outPoints->height = 1;
	  outPoints->width = 51200;/*51200 points per scan*/
	  outPoints->is_dense = false;
	  outPoints->resize(outPoints->height * outPoints->width);
  }
  else if(device_type_ == zvision::ML30B1){
	  outPoints->height = 1;
	  outPoints->width = 30000;/*30000 points per scan*/
	  outPoints->is_dense = false;
      outPoints->resize(outPoints->height * outPoints->width);
  }
  else if(device_type_ == zvision::MLX){
      outPoints->height = 1;
      outPoints->width = 96000;/*96000 points per scan*/
      outPoints->is_dense = false;
      outPoints->resize(outPoints->height * outPoints->width);
  }
  else if(device_type_ == zvision::MLXA1){
      outPoints->height = 1;
      outPoints->width = 114000;/*114000 points per scan*/
      outPoints->is_dense = false;
      outPoints->resize(outPoints->height * outPoints->width);
  }
  else if(device_type_ == zvision::MLXS){
      outPoints->height = 1;
      outPoints->width = 108000;/*108000 points per scan*/
      outPoints->is_dense = false;
      outPoints->resize(outPoints->height * outPoints->width);
  }

  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
      data_->unpack(scanMsg->packets[i], outPoints);
  }

  if((true == data_->use_lidar_time_) && (scanMsg->packets.size()))
  {
      data_->getTimeStampFromUdpPkt(scanMsg->packets[0], time_from_pkt_s, time_from_pkt_us);
      outPoints->header.stamp = time_from_pkt_s * 1000000 + time_from_pkt_us;
  }

//  sensor_msgs::PointCloud2 outMsg;
  auto outMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  sampled_cloud->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  sampled_cloud->header.frame_id = scanMsg->header.frame_id;
  if(DownsampleType::Voxel == downsample_type_)
  {
      voxel_grid_filter_.setInputCloud(outPoints);
      voxel_grid_filter_.filter(*sampled_cloud);
      pcl::toROSMsg(*sampled_cloud, *outMsg);
  }
  else if(DownsampleType::Line == downsample_type_)
  {
      sampled_cloud->resize(outPoints->size());
      int valid = 0;
      int line_interval = line_sample_;
      std::vector<int>& point_line_number = data_->point_line_number_;
      for(int p = 0; p < outPoints->size(); p++)
      {
          if(0 == (point_line_number[p] % (line_interval + 1)))
              sampled_cloud->at(valid++) = outPoints->at(p);
      }
      sampled_cloud->height = 1;
      sampled_cloud->width = valid;
      sampled_cloud->is_dense = false;
      sampled_cloud->resize(valid);
      pcl::toROSMsg(*sampled_cloud, *outMsg);
  }
  else
  {
      pcl::toROSMsg(*outPoints, *outMsg);
  }

    output_->publish(*outMsg);


}

}  // namespace zvision_lidar_pointcloud

RCLCPP_COMPONENTS_REGISTER_NODE(zvision_lidar_pointcloud::Convert)