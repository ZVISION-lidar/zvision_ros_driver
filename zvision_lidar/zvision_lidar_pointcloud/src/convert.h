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
#include "zvision_lidar_point_cloud_type.h"
#include "pointcloud_status.h"

namespace zvision_lidar_pointcloud
{
class Convert
{
public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~Convert()
  {
  }

  enum DownsampleType
  {
      None = 0,
      Voxel = 1,
      Line = 2,
      ConfigFile = 3,
  };
  uint8_t color_table[256*4] ={
    0,126,174,
    4,129,177,
    8,132,180,
    12,135,184,
    16,138,187,
    20,141,190,
    24,144,194,
    28,147,197,
    32,150,200,
    36,153,204,
    40,156,207,
    44,159,210,
    49,162,214,
    53,165,217,
    57,168,221,
    61,171,224,
    65,174,227,
    69,177,231,
    74,180,234,
    85,183,235,
    95,187,236,
    106,190,237,
    116,194,238,
    127,197,240,
    138,200,241,
    148,204,242,
    157,206,234,
    165,207,217,
    173,207,200,
    181,208,183,
    189,209,166,
    197,210,149,
    204,211,132,
    212,211,115,
    220,212,98,
    228,213,81,
    236,214,64,
    244,215,47,
    252,215,30,
    254,215,30,
    254,215,36,
    254,215,43,
    254,215,49,
    254,215,55,
    254,215,62,
    254,214,68,
    254,214,74,
    254,214,80,
    254,214,87,
    254,214,93,
    254,214,99,
    255,214,106,
    255,213,105,
    255,213,105,
    255,213,104,
    255,213,104,
    255,212,103,
    255,212,103,
    255,212,102,
    255,212,102,
    255,211,101,
    255,211,101,
    255,211,100,
    255,211,100,
    255,210,99,
    255,210,99,
    255,210,98,
    255,210,98,
    255,210,98,
    255,209,97,
    255,209,97,
    255,209,96,
    255,209,96,
    255,208,95,
    255,208,95,
    255,208,94,
    255,208,94,
    254,206,92,
    254,204,89,
    254,202,86,
    253,200,84,
    253,198,81,
    252,196,78,
    252,194,75,
    252,192,72,
    251,190,70,
    251,188,67,
    250,186,64,
    250,184,61,
    250,182,58,
    249,180,56,
    249,178,53,
    248,176,50,
    248,174,47,
    248,172,44,
    247,170,42,
    247,168,39,
    246,166,36,
    246,164,33,
    246,162,30,
    245,160,28,
    245,158,25,
    244,156,22,
    244,154,19,
    244,152,16,
    243,150,14,
    243,148,11,
    243,146,8,
    242,144,5,
    242,142,2,
    241,140,1,
    241,140,1,
    241,140,1,
    241,139,1,
    241,139,1,
    241,138,1,
    241,138,1,
    241,138,1,
    241,137,1,
    240,137,1,
    240,136,1,
    240,136,1,
    240,136,1,
    240,135,1,
    240,135,1,
    240,134,1,
    240,134,1,
    240,134,1,
    239,133,1,
    239,132,1,
    239,131,1,
    238,130,1,
    238,128,1,
    238,127,1,
    237,126,1,
    237,125,1,
    237,124,1,
    236,123,1,
    236,122,1,
    236,121,1,
    235,119,1,
    235,118,1,
    235,117,1,
    234,116,1,
    234,115,1,
    234,114,1,
    233,113,1,
    233,113,1,
    232,113,1,
    232,112,1,
    232,112,1,
    231,112,1,
    231,112,1,
    230,111,1,
    230,111,1,
    230,111,1,
    229,111,1,
    229,110,1,
    229,110,1,
    228,110,1,
    228,109,1,
    227,109,1,
    227,109,1,
    227,109,1,
    226,108,1,
    225,106,1,
    225,105,1,
    224,104,1,
    224,103,1,
    223,102,1,
    222,101,1,
    222,99,1,
    221,98,1,
    220,97,1,
    220,96,1,
    219,95,1,
    219,93,1,
    218,92,1,
    217,91,1,
    217,90,1,
    216,89,1,
    216,88,1,
    215,87,1,
    214,87,1,
    213,87,1,
    213,86,1,
    212,86,1,
    211,86,1,
    210,85,1,
    210,85,1,
    209,85,1,
    208,85,1,
    207,84,1,
    207,84,1,
    206,84,1,
    205,83,1,
    205,83,1,
    204,83,1,
    203,83,1,
    202,82,1,
    202,82,1,
    201,82,1,
    200,81,0,
    199,81,0,
    197,80,0,
    196,80,0,
    195,79,0,
    194,78,0,
    192,78,0,
    191,77,0,
    190,77,0,
    189,76,0,
    187,76,0,
    186,75,0,
    185,74,0,
    184,74,0,
    183,73,0,
    181,73,0,
    180,72,0,
    179,72,0,
    179,72,0,
    178,71,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    179,72,0,
    182,70,0,
    186,67,0,
    190,65,0,
    194,62,0,
    198,60,0,
    202,58,0,
    206,55,0,
    210,53,0,
    214,51,0,
    218,48,0,
    222,46,0,
    226,44,0,
    230,41,0,
    234,39,0,
    238,37,0,
    242,34,0,
    246,32,0,
    250,30,0
  };
private:

  void callback(zvision_lidar_pointcloud::CloudNodeConfig& config, uint32_t level);

  void processScan(const zvision_lidar_msgs::zvisionLidarScan::ConstPtr& scanMsg);

  int* get_nearest_point_index();

  double calDistance(pcl::PointXYZI a, pcl::PointXYZI b);

	uint8_t hex2uint8(char c);

  void getDownSampleMaskFromFile(std::string cfg_path);

  /// Pointer to dynamic reconfigure service srv_
  /**/std::shared_ptr<dynamic_reconfigure::Server<zvision_lidar_pointcloud::CloudNodeConfig> > srv_;

  zvision::LidarType device_type_;
  /**/std::shared_ptr<zvision_lidar_rawdata::RawData> data_;
  ros::Subscriber zvision_lidar_scan_;
  ros::Publisher out_info_;
  ros::Publisher output_;
  ros::Publisher output_colored_;
  ros::Publisher out_xyzirt_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  bool filter_enable_;
  bool use_outlier_removal_;
  double outlier_th_;
  bool pub_info_;
  bool pub_xyzi_;
  bool pub_colored_;
  bool pub_xyzirt_;
  bool pub_cfg_srv_;
  float leaf_size_;
  int line_sample_;
  int* nearest_table_;
  DownsampleType downsample_type_;

  bool use_pointcloud_status_diagnose_;
  int roi_sample_lines_;
  int roi_interval_;
  int min_roi_pointnum_;
  double z_height_;
  int statistic_frame_num_;
  double roi_z_diff_threshold_;
  double error_rate_threshold_;

  std::shared_ptr<std::vector<bool>> downsample_mask_ = nullptr;
  std::shared_ptr<PointCloudStatus> pointcloud_status_ptr_ = nullptr;
};
}  // namespace zvision_lidar_pointcloud
#endif
