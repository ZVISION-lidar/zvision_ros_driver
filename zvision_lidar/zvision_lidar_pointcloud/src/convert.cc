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
#include <pcl_conversions/pcl_conversions.h>

namespace zvision_lidar_pointcloud
{
std::string model;

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new zvision_lidar_rawdata::RawData())
{

  private_nh.param("model", model, std::string("ML30S-A1"));
  output_ = node.advertise<sensor_msgs::PointCloud2>("zvision_lidar_points", 20);

  //srv_question
  srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_pointcloud::CloudNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<zvision_lidar_pointcloud::CloudNodeConfig>::CallbackType f;

  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  data_->loadConfigFile(node, private_nh);
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


  if(model == "ML30S-A1"){
	  outPoints->height = 1;
	  outPoints->width = 51200;/*51200 points per scan*/
	  outPoints->is_dense = false;
	  outPoints->resize(outPoints->height * outPoints->width);


	  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
	  {
		  data_->unpack(scanMsg->packets[i], outPoints);
	  }

	  if((0 != data_->timestamp_type) && (scanMsg->packets.size()))
	  {
		  data_->getTimeStampFromUdpPkt(scanMsg->packets[0], time_from_pkt_s, time_from_pkt_us);
		  outPoints->header.stamp = time_from_pkt_s * 1000000 + time_from_pkt_us;
		  //outPoints->header.stamp = time_from_pkt_us * 1000;
	  }

	  sensor_msgs::PointCloud2 outMsg;
	  pcl::toROSMsg(*outPoints, outMsg);

	  output_.publish(outMsg);
  }
  else if(model == "ML30"){
	  outPoints->height = 1;
	  outPoints->width = 30000;/*30000 points per scan*/
	  outPoints->is_dense = false;
	  outPoints->resize(outPoints->height * outPoints->width);

	  // process each packet provided by the driver
	  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
	  {
		  data_->unpack(scanMsg->packets[i], outPoints);
	  }

	  if((0 != data_->timestamp_type) && (scanMsg->packets.size()))
	  {
		  data_->getTimeStampFromUdpPkt(scanMsg->packets[0], time_from_pkt_s, time_from_pkt_us);
		  outPoints->header.stamp = time_from_pkt_s * 1000000 + time_from_pkt_us;
		  //outPoints->header.stamp = time_from_pkt_us * 1000;
	  }

	  sensor_msgs::PointCloud2 outMsg;
	  pcl::toROSMsg(*outPoints, outMsg);

	  output_.publish(outMsg);

  }
}

}  // namespace zvision_lidar_pointcloud
