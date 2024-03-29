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
#include "tools/tools.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
namespace zvision_lidar_pointcloud
{
std::string model;


double Convert::calDistance(pcl::PointXYZI a, pcl::PointXYZI b){
    if(a.x == std::numeric_limits<float>::quiet_NaN() || b.x == std::numeric_limits<float>::quiet_NaN()) return std::numeric_limits<double>::infinity();
    return pow((a.x-b.x),2) + pow((a.y-b.y),2) + pow((a.z-b.z),2);
}
//根据角度文件生成固定距离distance 的整帧点云数据。通过KD树获取near_cnt个最近邻点的索引。
int* Convert::get_nearest_point_index()
{
    const int near_cnt = 8;
    static int g_neighborhood_index_ml30s[51200 * (near_cnt)];
    static int g_neighborhood_index_mlxs[108000 * (near_cnt)];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& point_cal : data_->cal_lut_->data)
    {
        pcl::_PointXYZ point_data;
        float distance = 2.0;
        point_data.x = distance * point_cal.cos_ele * point_cal.sin_azi;/*x*/
        point_data.y = distance * point_cal.cos_ele * point_cal.cos_azi;/*y*/
        point_data.z = distance * point_cal.sin_ele;/*z*/
        cloud->push_back(point_data);
    }
    pcl::search::Search<pcl::PointXYZ>::Ptr tree_;
    tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
#if 1

    tree_->setInputCloud(cloud);

    std::vector<int> nn_indices(near_cnt);
    std::vector<float> nn_dists(near_cnt);

    int cnt = 0;
    if(cloud->points.size()==51200){
        for (auto& pp : cloud->points)
        {
            if (tree_->nearestKSearch(pp, near_cnt, nn_indices, nn_dists) != near_cnt)
            {
                // distances[cp] = 0;
                // PCL_WARN("[pcl::%s::applyFilter] Searching for the closest %d neighbors failed.\n", getClassName().c_str(), mean_k_);
                // continue;
            }
            else
            {
                for (int i = 0; i < near_cnt; i++)
                {
                    g_neighborhood_index_ml30s[cnt * near_cnt + i] = nn_indices[i];
                    // nearstIndexTabel.push_back(nn_indices[i]);
                }
            }
            cnt++;
        }
        ROS_INFO("get_nearest_point_index  30s finished.\n");
        return g_neighborhood_index_ml30s;    
    }
    else if(cloud->points.size()==108000){
        for (auto& pp : cloud->points)
        {
            if (tree_->nearestKSearch(pp, near_cnt, nn_indices, nn_dists) != near_cnt)
            {
                //distances[cp] = 0;
                //PCL_WARN("[pcl::%s::applyFilter] Searching for the closest %d neighbors failed.\n", getClassName().c_str(), mean_k_);
                //continue;
            }
            else
            {
                for (int i = 0; i < near_cnt; i++)
                {
                    g_neighborhood_index_mlxs[cnt * near_cnt + i] = nn_indices[i];
                    // nearstIndexTabel.push_back(nn_indices[i]);
                }
            }
            cnt++;
        }
        ROS_INFO("get_nearest_point_index xs finished.\n");

        return g_neighborhood_index_mlxs;    
    }
    
#endif
    ROS_INFO("get_nearest_point_index failed.\n");
    return nullptr;
}

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new zvision_lidar_rawdata::RawData())
  ,filter_enable_(false)
{

  private_nh.param("model", model, std::string("ML30SA1"));
  device_type_ = zvision::LidarTools::GetDeviceTypeFromTypeString(model);
  //private_nh.param("filter_enable", filter_enable_, false);

  std::string downsample_string = "";
  private_nh.param("downsample_type", downsample_string, std::string(""));
  if(downsample_string == "downsample_line")
  {
      private_nh.param("line_sample", line_sample_, 2);
      this->downsample_type_ = DownsampleType::Line;
      ROS_INFO("Downsample type is [Line], 1 in %d.", line_sample_);
  }
  else if(downsample_string == "downsample_voxel")
  {
      private_nh.param("voxel_leaf_size", leaf_size_, 0.2f);
      voxel_grid_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      this->downsample_type_ = DownsampleType::Voxel;
      ROS_INFO("Downsample type is [Voxel], leaf size is [%.3f].", leaf_size_);
  }
  else
  {
      this->downsample_type_ = DownsampleType::None;
      ROS_INFO("Downsample type is [None] publish raw pointcloud.");
  }
  private_nh.param("use_outlier_removal", use_outlier_removal_, false);
  ROS_INFO("use_outlier_removal: %c", use_outlier_removal_?'y':'n');
  private_nh.param("outlier_th", outlier_th_, 0.25);
  ROS_INFO("Outlier points threshold  is, [%.3f].", outlier_th_);
  private_nh.param("use_blue_gold_color_scheme", pub_colored_, false);
  ROS_INFO("use_blue_gold_color_scheme: %c", pub_colored_?'y':'n');
  output_ = node.advertise<sensor_msgs::PointCloud2>("zvision_lidar_points", 20);
  output_colored_ = node.advertise<sensor_msgs::PointCloud2>("zvision_lidar_points_colored", 20);

  //srv_question
  srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_pointcloud::CloudNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<zvision_lidar_pointcloud::CloudNodeConfig>::CallbackType f;

  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  data_->loadConfigFile(node, private_nh);
  nearest_table_ = nullptr;
//   if(data_->cal_init_ok_) nearest_table_ = get_nearest_point_index();
  // subscribe to zvisionlidarScan packets
  zvision_lidar_scan_ = node.subscribe("zvision_lidar_packets", 20, &Convert::processScan, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true));

}

void Convert::callback(zvision_lidar_pointcloud::CloudNodeConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  printf("parameter:%f %f %f %f %f %f\n", config.x_trans, config.y_trans,config.z_trans,config.z_rotation,config.y_rotation,config.z_rotation);
  data_->x_trans = config.x_trans;
  data_->y_trans = config.y_trans;
  data_->z_trans = config.z_trans;
  data_->x_rotation = config.x_rotation / 180.0 * M_PI;
  data_->y_rotation = config.y_rotation / 180.0 * M_PI;
  data_->z_rotation = config.z_rotation / 180.0 * M_PI;
  // config_.time_offset = config.time_offset;
}





/** @brief Callback for raw scan messages. *///IMPORTANT
void Convert::processScan(const zvision_lidar_msgs::zvisionLidarScan::ConstPtr& scanMsg)
{
  long long time_from_pkt_s = 0;
  long long time_from_pkt_us = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();

  if(!data_->isCalibrationInitOk())
  {
      ROS_WARN_ONCE("Calibration data is not initialized.");
      return;
  }
  else
      ROS_INFO_ONCE("Calibration data init ok.");

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

  pcl::PointCloud<pcl::PointXYZI>::Ptr  outputPtr(new pcl::PointCloud<pcl::PointXYZI>);
  outputPtr->resize(outPoints->size());
  outputPtr->header = outPoints->header;
  if(nearest_table_ == nullptr && data_->cal_init_ok_){
    nearest_table_ = get_nearest_point_index();
  }
  if(use_outlier_removal_ && nearest_table_ != nullptr){
    pcl::PointXYZI point_nan;
    point_nan.x = std::numeric_limits<float>::quiet_NaN();
    point_nan.y = std::numeric_limits<float>::quiet_NaN();
    point_nan.z = std::numeric_limits<float>::quiet_NaN();
    point_nan.intensity = 0;
    size_t point_valid = outPoints->size();
    if(point_valid==108000 || point_valid==51200) {
        for(size_t i = 0 ; i < point_valid; ++i){
            size_t valid_neighbor = 0;
            for(size_t j = (i * 8 + 1); j < (i * 8 + 8); ++j){
                if(calDistance(outPoints->at(i),outPoints->at(nearest_table_[j])) < outlier_th_){
                    valid_neighbor++;
                }
            }
            if(valid_neighbor >= 2){
                    outputPtr->at(i) = outPoints->at(i);
            }
            else {
                outputPtr->at(i) = point_nan;
            }
        }
    }
    else outputPtr = outPoints;
  }
  else outputPtr = outPoints;


  sensor_msgs::PointCloud2 outMsg;
  sensor_msgs::PointCloud2 outMsgColored;
  pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  sampled_cloud->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  sampled_cloud->header.frame_id = scanMsg->header.frame_id;
  if(DownsampleType::Voxel == downsample_type_)
  {
      voxel_grid_filter_.setInputCloud(outputPtr);
      voxel_grid_filter_.filter(*sampled_cloud);
      pcl::toROSMsg(*sampled_cloud, outMsg);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sampled_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGBA>);
      sampled_cloud_colored->resize(sampled_cloud->size());
      sampled_cloud_colored->header.stamp = sampled_cloud->header.stamp;
      sampled_cloud_colored->header.frame_id = sampled_cloud->header.frame_id;
      for(int i = 0; i < sampled_cloud_colored->size();++i){
        sampled_cloud_colored->at(i).x = sampled_cloud->at(i).x; 
        sampled_cloud_colored->at(i).y = sampled_cloud->at(i).y; 
        sampled_cloud_colored->at(i).z = sampled_cloud->at(i).z; 
        sampled_cloud_colored->at(i).r = color_table[(int)outputPtr->at(i).intensity *3 + 0]; 
        sampled_cloud_colored->at(i).g = color_table[(int)outputPtr->at(i).intensity *3 + 1]; 
        sampled_cloud_colored->at(i).b = color_table[(int)outputPtr->at(i).intensity *3 + 2];  
      }
      pcl::toROSMsg(*sampled_cloud_colored,outMsgColored);
  }
  else if(DownsampleType::Line == downsample_type_)
  {
      sampled_cloud->resize(outputPtr->size());
      int valid = 0;
      int line_interval = line_sample_;
      std::vector<int>& point_line_number = data_->point_line_number_;
      for(int p = 0; p < outputPtr->size(); p++)
      {
          if(0 == (point_line_number[p] % (line_interval + 1)))
              sampled_cloud->at(valid++) = outputPtr->at(p);
      }
      sampled_cloud->height = 1;
      sampled_cloud->width = valid;
      sampled_cloud->is_dense = false;
      sampled_cloud->resize(valid);
      pcl::toROSMsg(*sampled_cloud, outMsg);

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sampled_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGBA>);
      sampled_cloud_colored->resize(sampled_cloud->size());
      sampled_cloud_colored->header.stamp = sampled_cloud->header.stamp;
      sampled_cloud_colored->header.frame_id = sampled_cloud->header.frame_id;
      for(int i = 0; i < sampled_cloud_colored->size();++i){
        sampled_cloud_colored->at(i).x = sampled_cloud->at(i).x; 
        sampled_cloud_colored->at(i).y = sampled_cloud->at(i).y; 
        sampled_cloud_colored->at(i).z = sampled_cloud->at(i).z; 
        sampled_cloud_colored->at(i).r = color_table[(int)outputPtr->at(i).intensity *3 + 0]; 
        sampled_cloud_colored->at(i).g = color_table[(int)outputPtr->at(i).intensity *3 + 1]; 
        sampled_cloud_colored->at(i).b = color_table[(int)outputPtr->at(i).intensity *3 + 2];  
      }
      pcl::toROSMsg(*sampled_cloud_colored,outMsgColored);
  }
  else
  {
      pcl::toROSMsg(*outputPtr, outMsg);

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sampled_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGBA>);
      sampled_cloud_colored->resize(outputPtr->size());
      sampled_cloud_colored->header.stamp = outputPtr->header.stamp;
      sampled_cloud_colored->header.frame_id = outputPtr->header.frame_id;
      for(int i = 0; i < sampled_cloud_colored->size();++i){
        sampled_cloud_colored->at(i).x = outputPtr->at(i).x; 
        sampled_cloud_colored->at(i).y = outputPtr->at(i).y; 
        sampled_cloud_colored->at(i).z = outputPtr->at(i).z; 
        sampled_cloud_colored->at(i).r = color_table[(int)outputPtr->at(i).intensity *3 + 0]; 
        sampled_cloud_colored->at(i).g = color_table[(int)outputPtr->at(i).intensity *3 + 1]; 
        sampled_cloud_colored->at(i).b = color_table[(int)outputPtr->at(i).intensity *3 + 2];  
      }
      pcl::toROSMsg(*sampled_cloud_colored,outMsgColored);
  }

  output_.publish(outMsg);
  if(pub_colored_) output_colored_.publish(outMsgColored);
  



   

}

}  // namespace zvision_lidar_pointcloud
