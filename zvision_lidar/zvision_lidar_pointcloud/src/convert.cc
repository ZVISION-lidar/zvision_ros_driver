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
#include <fstream>
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
  else if(downsample_string == "downsample_cfg_file"){
    std::string cfg_path;
    private_nh.param("downsample_cfg_path", cfg_path, std::string(""));
    if(!cfg_path.empty()){
      ROS_INFO("Downsample type is [ConfigFile].");
      ROS_INFO("downsample_cfg_path : [%s].", cfg_path.c_str());
      getDownSampleMaskFromFile(cfg_path);
      this->downsample_type_ = DownsampleType::ConfigFile;
    }
  }
  else
  {
      this->downsample_type_ = DownsampleType::None;
      ROS_INFO("Downsample type is [None] publish raw pointcloud.");
  }
  private_nh.param("use_outlier_removal", use_outlier_removal_, false);
  ROS_INFO("use_outlier_removal: %c", use_outlier_removal_?'y':'n');
  private_nh.param("outlier_th", outlier_th_, 0.05);
  ROS_INFO("Outlier points threshold is, [%.3f].", outlier_th_);
  
  private_nh.param("publisher_frame_info", pub_info_, true);
  ROS_INFO("publisher_frame_info: %c", pub_info_?'y':'n');
  if(pub_info_)
    out_info_ = node.advertise<zvision_lidar_msgs::zvisionLidarInformation>("zvision_lidar_frame_info", 20);

  private_nh.param("publisher_pointcloud_type_xyzi", pub_xyzi_, true);
  ROS_INFO("publisher_pointcloud_type_xyzi: %c", pub_xyzi_?'y':'n');
  if(pub_xyzi_) 
    output_ = node.advertise<sensor_msgs::PointCloud2>("zvision_lidar_points", 20);

  private_nh.param("use_blue_gold_color_scheme", pub_colored_, false);
  ROS_INFO("use_blue_gold_color_scheme: %c", pub_colored_?'y':'n');
  if(pub_colored_)
    output_colored_ = node.advertise<sensor_msgs::PointCloud2>("zvision_lidar_points_colored", 20);

  private_nh.param("publisher_pointcloud_type_xyzirt", pub_xyzirt_, false);
  ROS_INFO("publisher_pointcloud_type_xyzirt: %c", pub_xyzirt_?'y':'n');
  if(pub_xyzirt_)
    out_xyzirt_ = node.advertise<sensor_msgs::PointCloud2>("zvision_lidar_points_xyzirt", 20);

  // lidar status
  private_nh.param("use_pointcloud_status_diagnose", use_pointcloud_status_diagnose_, false);
  ROS_INFO("use_pointcloud_status_diagnose: %c", use_pointcloud_status_diagnose_?'y':'n');

  if(use_pointcloud_status_diagnose_){
    private_nh.param("statistic_frame_num", statistic_frame_num_, 600);
    ROS_INFO("Pointcloud_status statistic frame_num is, [%d].", statistic_frame_num_);
    
    private_nh.param("roi_sample_lines", roi_sample_lines_, 10);
    ROS_INFO("Pointcloud_status roi_sample_lines is, [%d].", roi_sample_lines_);

    private_nh.param("roi_interval", roi_interval_, 10);
    ROS_INFO("Pointcloud_status roi_interval is, [%d].", roi_interval_);

    private_nh.param("min_roi_pointnum", min_roi_pointnum_, 60);
    ROS_INFO("Pointcloud_status min_roi_pointnum is, [%d].", min_roi_pointnum_);

    private_nh.param("z_height", z_height_, -0.58);
    ROS_INFO("Pointcloud_status z_height is, [%.3f].", z_height_);

    private_nh.param("roi_z_diff_threshold", roi_z_diff_threshold_, 0.1);
    ROS_INFO("Pointcloud_status roi_z_diff_threshold  is, [%.3f].", roi_z_diff_threshold_);

    private_nh.param("error_rate_threshold", error_rate_threshold_, 0.5);
    ROS_INFO("Pointcloud_status statistic error_rate_threshold is, [%.3f].", error_rate_threshold_);

    pointcloud_status_ptr_.reset(new PointCloudStatus(
      statistic_frame_num_,
      roi_sample_lines_, 
      roi_interval_,
      min_roi_pointnum_,
      z_height_,
      roi_z_diff_threshold_,
      error_rate_threshold_
    ));
  } 
  //srv_question
  private_nh.param("dynamic_reconfigure_server", pub_cfg_srv_, true);
  ROS_INFO("dynamic_reconfigure_server: %c", pub_cfg_srv_?'y':'n');
  if(pub_cfg_srv_){
    srv_ = /**/std::make_shared<dynamic_reconfigure::Server<zvision_lidar_pointcloud::CloudNodeConfig> >(private_nh);
    srv_->setCallback(std::bind(&Convert::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

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
  if(use_pointcloud_status_diagnose_){

    printf("pointcloud_status parameter:%d %d %d %d %f %f %f\n", 
        config.roi_sample_lines, 
        config.roi_interval,
        config.statistic_frame_num,
        config.min_roi_pointnum,
        config.z_height,
        config.roi_z_diff_threshold,
        config.error_rate_threshold);
    pointcloud_status_ptr_->set_threshold(
        config.statistic_frame_num,
        config.roi_sample_lines,
        config.roi_interval,
        config.min_roi_pointnum,
        config.z_height,
        config.roi_z_diff_threshold,
        config.error_rate_threshold
    );
  }
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
    // for downsample mode
      if(scanMsg->packets.size() == 80){
          outPoints->width = 25600;
          downsample_type_ = DownsampleType::None;
      }else if(scanMsg->packets.size() == 40){
          outPoints->width = 12800;
          downsample_type_ = DownsampleType::None;
      }
	  outPoints->is_dense = false;
	  outPoints->resize(outPoints->height * outPoints->width);
  }
  else if(device_type_ == zvision::ML30SPlusA1 || device_type_ == zvision::ML30SPlusB1){
	  outPoints->height = 1;
	  outPoints->width = 51200;
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

  // get lidar first firing point timestamp
  if((true == data_->use_lidar_time_) && (scanMsg->packets.size()))
  {
      data_->getTimeStampFromUdpPkt(scanMsg->packets[0], time_from_pkt_s, time_from_pkt_us);
      outPoints->header.stamp = time_from_pkt_s * 1000000 + time_from_pkt_us;
  }

  // parsing packets
  pcl::PointCloud<ZvPointXYZIRT>::Ptr points_xyzirt(new pcl::PointCloud<ZvPointXYZIRT>);
  points_xyzirt->resize(outPoints->size());
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
      data_->unpack(scanMsg->packets[i], points_xyzirt);
  }

  for(int i = 0;i<outPoints->size();i++){
    outPoints->at(i).x = points_xyzirt->at(i).x;
    outPoints->at(i).y = points_xyzirt->at(i).y;
    outPoints->at(i).z = points_xyzirt->at(i).z;
    outPoints->at(i).intensity = points_xyzirt->at(i).intensity;
  }
  
  // publish lidar information
  if(pub_info_)
  {
    zvision_lidar_msgs::zvisionLidarInformationPtr info(new zvision_lidar_msgs::zvisionLidarInformation);
    // get lidar first udp packet timestamp from udp data
    bool is_ptp = false;
    bool is_lock = false;
    data_->getLockStatusFromUdpPkt(scanMsg->packets[0],is_ptp,  is_lock);
    info->lock_status = is_lock;
    info->is_ptp = is_ptp;
    float bias = .0f;
    data_->getApdBiasFromUdpPkt(scanMsg->packets[0], bias);
    info->apd_bias = bias;
    uint32_t sec = outPoints->header.stamp / 1000000;
    uint32_t nsec = (outPoints->header.stamp%1000000) * 1000;
    info->stamp = ros::Time(sec, nsec);
    if(use_pointcloud_status_diagnose_){
      info->is_pc_status_error = pointcloud_status_ptr_->check_status(outPoints);
    }
    out_info_.publish(info);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr  outputPtr(new pcl::PointCloud<pcl::PointXYZI>);
  outputPtr->resize(outPoints->size());
  outputPtr->header = outPoints->header;
  if(nearest_table_ == nullptr && data_->cal_init_ok_ &&(outPoints->size() == 108000 || outPoints->size() == 51200)){
    nearest_table_ = get_nearest_point_index();
  }
  if(use_outlier_removal_ && nearest_table_ != nullptr){
    pcl::PointXYZI point_nan;
    point_nan.x = std::numeric_limits<float>::quiet_NaN();
    point_nan.y = std::numeric_limits<float>::quiet_NaN();
    point_nan.z = std::numeric_limits<float>::quiet_NaN();
    point_nan.intensity = 0;
    size_t point_valid = outPoints->size();

    size_t invaild_point_num = 0;
    size_t nan_point_num = 0;
    double dis_coeff = powf(outlier_th_, 2);
    if (point_valid == 108000 || point_valid == 51200)
    {
    for (size_t i = 0; i < point_valid; ++i)
    {
        size_t valid_neighbor = 0;

        // 无效点与0距离跳过
        if (std::isnan(outPoints->at(i).x))
        {
        nan_point_num++;
        continue;
        }
        if (outPoints->at(i).x == 0 && outPoints->at(i).y == 0 && outPoints->at(i).z == 0)
        {
        outputPtr->at(i) = point_nan;
        nan_point_num++;
        continue;
        }

        // 动态阈值计算
        auto outlier_th_square = dis_coeff * (powf(outPoints->at(i).x, 2) + powf(outPoints->at(i).y, 2) + powf(outPoints->at(i).z, 2));
        for (size_t j = (i * 8); j < (i * 8 + 8); j++)
        {
        double _distance = calDistance(outPoints->at(i), outPoints->at(nearest_table_[j]));
        if (_distance < outlier_th_square)
        {
            valid_neighbor++;
            if (valid_neighbor >= 2)
            break;
        }
        }

        // 离群值判断
        if (valid_neighbor >= 2)
        {
        outputPtr->at(i) = outPoints->at(i);
        }
        else
        {
        outputPtr->at(i) = point_nan;
        invaild_point_num++;
        }
    }
    std::cout << "invalid point : " << nan_point_num << std::endl;
    std::cout << "deleted point : " << invaild_point_num << std::endl;
    ROS_INFO("point sum is %d", outPoints->size());
    }
    else outputPtr = outPoints;
  }
  else outputPtr = outPoints;

  sensor_msgs::PointCloud2 outMsg;
  sensor_msgs::PointCloud2 outMsgColored;
  sensor_msgs::PointCloud2 out_msg_xyzirt;

  // pointcloud downsampling
  pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  sampled_cloud->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  sampled_cloud->header.frame_id = scanMsg->header.frame_id;
  // point indices
  std::vector<int> index_used;
  index_used.resize(outputPtr->size(), -1);
  for(int i = 0;i<outputPtr->size();i++)
    index_used.at(i) = i;

  if(DownsampleType::Voxel == downsample_type_)
  {
      /* update index_used */
      if(pub_xyzirt_){
        sampled_cloud->points = outputPtr->points;
      }
      else{
        voxel_grid_filter_.setInputCloud(outputPtr);
        voxel_grid_filter_.filter(*sampled_cloud);
      }

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
      // update index_used
      index_used.resize(outputPtr->size(), -1);
      for(int p = 0; p < outputPtr->size(); p++)
      {
          if(0 == (point_line_number[p] % (line_interval + 1))){
              index_used.at(valid) = p;
              sampled_cloud->at(valid++) = outputPtr->at(p);
          }
      }
      sampled_cloud->height = 1;
      sampled_cloud->width = valid;
      sampled_cloud->is_dense = false;
      sampled_cloud->resize(valid);
      index_used.resize(valid);
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
  else if(DownsampleType::ConfigFile == downsample_type_){
    // filter pointcloud from config file
    if(downsample_mask_ && (downsample_mask_->size() == outPoints->size()) && (device_type_ == zvision::ML30SA1)){
      index_used.resize(outPoints->size(), -1);
      sampled_cloud->resize(outPoints->size());
      int id = 0;
      for(int it = 0;it<outPoints->size();it++){
        if(downsample_mask_->at(it)){
            sampled_cloud->at(id) = outPoints->at(it);
            index_used.at(id) = it;
            id++;
        }
      }
      sampled_cloud->height = 1;
      sampled_cloud->width = id;
      sampled_cloud->is_dense = false;
      sampled_cloud->resize(id);
       index_used.resize(id);
    }
    else
    {
      sampled_cloud->height = 1;
      sampled_cloud->width = outPoints->size();
      sampled_cloud->is_dense = false;
      sampled_cloud->resize(outPoints->size());
      for(int it = 0;it<outPoints->size();it++){
        sampled_cloud->at(it) = outPoints->at(it);
      }
    }

    if(pub_xyzi_)
      pcl::toROSMsg(*sampled_cloud, outMsg);

    if(pub_colored_){
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
  }
  else
  {
      if(pub_xyzi_)
        pcl::toROSMsg(*outputPtr, outMsg);

        if(pub_colored_){
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
  }

  if(pub_xyzi_) output_.publish(outMsg);

  if(pub_colored_) output_colored_.publish(outMsgColored);
  
  /* generate point xyzirt data */
  if(pub_xyzirt_){
    // we generate pointcloud xyzirt data
    pcl::PointCloud<pcl::PointXYZI>::Ptr dst_cloud  = nullptr;
    if(DownsampleType::None == downsample_type_ )
      dst_cloud = outputPtr;
    else
      dst_cloud = sampled_cloud;

    bool use_rt = false;
    if(index_used.size() == dst_cloud->size())
      use_rt=true;
    else    ROS_WARN_ONCE("Disable RT output for used index size [%lu] not matched with destination pointcloud size [%lu].",index_used.size(),dst_cloud->size());

    pcl::PointCloud<ZvPointXYZIRT>::Ptr dst_cloud_xyzirt(new pcl::PointCloud<ZvPointXYZIRT>);
    dst_cloud_xyzirt->resize(dst_cloud->size());
    dst_cloud_xyzirt->header.stamp = dst_cloud->header.stamp;
    dst_cloud_xyzirt->header.frame_id = dst_cloud->header.frame_id;
    for(int it = 0;it<dst_cloud->size();it++){
      ZvPointXYZIRT p;
      p.x = dst_cloud->at(it).x;
      p.y = dst_cloud->at(it).y;
      p.z = dst_cloud->at(it).z;
      p.intensity = (uint8_t)dst_cloud->at(it).intensity;

      // we now filter timestamp and fov id for filtered  point
      if(use_rt && (index_used[it] >= 0 )){
        p.ring = points_xyzirt->at(index_used[it]).ring;
        p.timestamp =  points_xyzirt->at(index_used[it]).timestamp;
      }
      else{
        p.ring = 0;
        p.timestamp =  dst_cloud->header.stamp;
      }
      dst_cloud_xyzirt->at(it) = p;
    }

    pcl::toROSMsg(*dst_cloud_xyzirt, out_msg_xyzirt);
    out_xyzirt_.publish(out_msg_xyzirt);
  }


}

uint8_t Convert::hex2uint8(char c) {

		uint8_t val = 0x0F;
		if (c >= 'A' && c <= 'Z')
			val = c - 'A' + 10;
		else if (c >= 'a' && c <= 'z')
			val = c - 'a' + 10;
		else if (c >= '0' && c <= '9')
			val = c - '0';
		return val;
	}

void Convert::getDownSampleMaskFromFile(std::string cfg_path) {

		if (cfg_path.empty())
			return;
    // for ml30s device, update cover table
		try {
      // load cfg file data
			std::ifstream in(cfg_path.c_str(), std::ios::in);
			if (!in.is_open()){
        ROS_WARN("Open downsample file failed.");
        return;
      }
      downsample_mask_.reset( new std::vector<bool>(51200,true));
			uint8_t flg = 0x80;
			uint8_t fov_in_group[8] = {0, 2, 4, 6, 5, 7, 1, 3};
			int total_cnt = 640;
			std::string line;
			int id = 0;
			while (std::getline(in, line))
			{
				// check enter
        if(line.at(line.size()-1) == 13){
            line = line.substr(0,line.size()-1);
        }
				if (line.size() != 20)
					continue;

				// get value
				uint8_t masks[10] = {0xFF};
				for (int b = 0; b < 10;b++) {
					char h = line.at(b * 2);
					char l = line.at(b * 2 + 1);
					masks [b] = hex2uint8(h) << 4 | hex2uint8(l);
				}

				// update
				for (int j = 0; j < 10; j++) {
					for (int k = 0; k < 8;k++) {
						flg = 0x80 >> k;
						if ((flg & masks[j]) != flg) {
							int fov = id / 80;
							int group = (id * 10 * 8 + j * 8 + k) % 6400;
							int point_id = group * 8 + fov_in_group[fov];
							downsample_mask_->at(point_id) = 0;
						}
					}
				}
				id++;
			}

			in.close();
			if (id != total_cnt) {
				downsample_mask_.reset();
			}
		}
		catch (std::exception e)
		{
			downsample_mask_.reset();
			return;
		}

	}
}  // namespace zvision_lidar_pointcloud
