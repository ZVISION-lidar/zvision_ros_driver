/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  Copyright (C) 2019 Zvision Technology, Pengfei Cui
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the ZVISION LIDAR 3D LIDARs
 */
#include "zvision_driver.h"
#include <zvision_lidar_msgs/zvisionLidarScan.h>

namespace zvision_lidar_driver
{
zvisionLidarDriver::zvisionLidarDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
{

  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("zvision_lidar"));

  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);


  private_nh.param("model", config_.model, std::string("ML30SA1"));

  int fre = 10;
  private_nh.param("frequency", fre, 10);
  double packet_rate_ML30 = 125 * fre;
  double packet_rate_ML30S_A1 = 160 * fre;
  double packet_rate_MLX = 400 * fre;
  double packet_rate_MLXA1 = 475 * fre;
  double packet_rate_MLXS = 450 * fre;
  std::string model_full_name;

  // product model
  if (config_.model == "ML30B1")
  {
    model_full_name = "ZVISION-LiDAR-ML30";
  }
  else if(config_.model == "ML30SA1"){
    model_full_name = "ZVISION-LiDAR-ML30SA1";
  }
  else if(config_.model == "MLX"){
    model_full_name = "ZVISION-LiDAR-MLX";
  }
  else if(config_.model == "MLXA1"){
    model_full_name = "ZVISION-LiDAR-MLXA1";
  }
  else if(config_.model == "MLXS"){
    model_full_name = "ZVISION-LiDAR-MLXS";
  }
  else
  {
    ROS_ERROR_STREAM("unknown LIDAR model: " << config_.model);
  }
  std::string deviceName(std::string("Zvision ") + model_full_name);


  if(config_.model == "ML30B1"){
	  int npackets = 125;
	  private_nh.param("npackets", config_.npackets, npackets);
	  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

	  std::string dump_file;
	  private_nh.param("pcap", dump_file, std::string(""));

	  int port_to_recv_udppkt;
	  private_nh.param("udp_port", port_to_recv_udppkt, (int)UDP_DATA_PORT_NUMBER);

	  // Initialize dynamic reconfigure
	  srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> >(private_nh);
	  dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig>::CallbackType f;
	  f = boost::bind(&zvisionLidarDriver::callback, this, _1, _2);
	  srv_->setCallback(f);  // Set callback function und call initially

	  // initialize diagnostics
	  diagnostics_.setHardwareID(deviceName);
	  const double diag_freq = packet_rate_ML30 / config_.npackets;
	  diag_max_freq_ = diag_freq;
	  diag_min_freq_ = diag_freq;

	  using namespace diagnostic_updater;
	  diag_topic_.reset(new TopicDiagnostic("zvision_lidar_packets", diagnostics_,
	                                        FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
	                                        TimeStampStatusParam()));

	  // open zvision lidar input device or file
	  if (dump_file != "")  // have PCAP file?
	  {
		  // read data from packet capture file
		  input_.reset(new zvision_lidar_driver::InputPCAP(private_nh, port_to_recv_udppkt, packet_rate_ML30, dump_file));
	  }
	  else
	  {
		  // read data from live socket
		  input_.reset(new zvision_lidar_driver::InputSocket(private_nh, port_to_recv_udppkt));
	  }
	  // raw packet output topic
	  output_ = node.advertise<zvision_lidar_msgs::zvisionLidarScan>("zvision_lidar_packets", 20);

  }
  else if(config_.model == "ML30SA1"){

	  int npackets = 160;
	  private_nh.param("npackets", config_.npackets, npackets);
	  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

	  std::string dump_file;
	  private_nh.param("pcap", dump_file, std::string(""));

	  int port_to_recv_udppkt;
	  private_nh.param("udp_port", port_to_recv_udppkt, (int)UDP_DATA_PORT_NUMBER);

	  // Initialize dynamic reconfigure
	  srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> >(private_nh);
	  dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig>::CallbackType f;
	  f = boost::bind(&zvisionLidarDriver::callback, this, _1, _2);
	  srv_->setCallback(f);  // Set callback function und call initially

	  // initialize diagnostics
	  diagnostics_.setHardwareID(deviceName);
      const double diag_freq = packet_rate_ML30S_A1 / config_.npackets;
	  diag_max_freq_ = diag_freq;
	  diag_min_freq_ = diag_freq;

	  using namespace diagnostic_updater;
	  diag_topic_.reset(new TopicDiagnostic("zvision_lidar_packets", diagnostics_,
	                                        FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
	                                        TimeStampStatusParam()));

	  // open zvision lidar input device or file
	  if (dump_file != "")  // have PCAP file?
	  {
		  // read data from packet capture file
		  input_.reset(new zvision_lidar_driver::InputPCAP(private_nh, port_to_recv_udppkt, packet_rate_ML30S_A1, dump_file));
	  }
	  else
	  {
		  // read data from live socket
		  input_.reset(new zvision_lidar_driver::InputSocket(private_nh, port_to_recv_udppkt));
	  }
	  // raw packet output topic
	  output_ = node.advertise<zvision_lidar_msgs::zvisionLidarScan>("zvision_lidar_packets", 20);
  }
  else if(config_.model == "MLX"){
      int npackets = 400;
      private_nh.param("npackets", config_.npackets, npackets);
      ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

      std::string dump_file;
      private_nh.param("pcap", dump_file, std::string(""));

      int port_to_recv_udppkt;
      private_nh.param("udp_port", port_to_recv_udppkt, (int)UDP_DATA_PORT_NUMBER);

      // Initialize dynamic reconfigure
      srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> >(private_nh);
      dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig>::CallbackType f;
      f = boost::bind(&zvisionLidarDriver::callback, this, _1, _2);
      srv_->setCallback(f);  // Set callback function und call initially

      // initialize diagnostics
      diagnostics_.setHardwareID(deviceName);
      const double diag_freq = packet_rate_MLX / config_.npackets;
      diag_max_freq_ = diag_freq;
      diag_min_freq_ = diag_freq;

      using namespace diagnostic_updater;
      diag_topic_.reset(new TopicDiagnostic("zvision_lidar_packets", diagnostics_,
                                            FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
                                            TimeStampStatusParam()));

      // open zvision lidar input device or file
      if (dump_file != "")  // have PCAP file?
      {
          // read data from packet capture file
          input_.reset(new zvision_lidar_driver::InputPCAP(private_nh, port_to_recv_udppkt, packet_rate_MLX, dump_file));
      }
      else
      {
          // read data from live socket
          input_.reset(new zvision_lidar_driver::InputSocket(private_nh, port_to_recv_udppkt));
      }
      // raw packet output topic
      output_ = node.advertise<zvision_lidar_msgs::zvisionLidarScan>("zvision_lidar_packets", 20);
  }
  else if(config_.model == "MLXA1"){
      int npackets = 475;
      private_nh.param("npackets", config_.npackets, npackets);
      ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

      std::string dump_file;
      private_nh.param("pcap", dump_file, std::string(""));

      int port_to_recv_udppkt;
      private_nh.param("udp_port", port_to_recv_udppkt, (int)UDP_DATA_PORT_NUMBER);

      // Initialize dynamic reconfigure
      srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> >(private_nh);
      dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig>::CallbackType f;
      f = boost::bind(&zvisionLidarDriver::callback, this, _1, _2);
      srv_->setCallback(f);  // Set callback function und call initially

      // initialize diagnostics
      diagnostics_.setHardwareID(deviceName);
      const double diag_freq = packet_rate_MLXA1 / config_.npackets;
      diag_max_freq_ = diag_freq;
      diag_min_freq_ = diag_freq;

      using namespace diagnostic_updater;
      diag_topic_.reset(new TopicDiagnostic("zvision_lidar_packets", diagnostics_,
                                            FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
                                            TimeStampStatusParam()));

      // open zvision lidar input device or file
      if (dump_file != "")  // have PCAP file?
      {
          // read data from packet capture file
          input_.reset(new zvision_lidar_driver::InputPCAP(private_nh, port_to_recv_udppkt, packet_rate_MLXA1, dump_file));
      }
      else
      {
          // read data from live socket
          input_.reset(new zvision_lidar_driver::InputSocket(private_nh, port_to_recv_udppkt));
      }
      // raw packet output topic
      output_ = node.advertise<zvision_lidar_msgs::zvisionLidarScan>("zvision_lidar_packets", 20);
  }
  else if(config_.model == "MLXS"){
      int npackets = 450;
      private_nh.param("npackets", config_.npackets, npackets);
      ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

      std::string dump_file;
      private_nh.param("pcap", dump_file, std::string(""));

      int port_to_recv_udppkt;
      private_nh.param("udp_port", port_to_recv_udppkt, (int)UDP_DATA_PORT_NUMBER);

      // Initialize dynamic reconfigure
      srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> >(private_nh);
      dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig>::CallbackType f;
      f = boost::bind(&zvisionLidarDriver::callback, this, _1, _2);
      srv_->setCallback(f);  // Set callback function und call initially

      // initialize diagnostics
      diagnostics_.setHardwareID(deviceName);
      const double diag_freq = packet_rate_MLXS / config_.npackets;
      diag_max_freq_ = diag_freq;
      diag_min_freq_ = diag_freq;

      using namespace diagnostic_updater;
      diag_topic_.reset(new TopicDiagnostic("zvision_lidar_packets", diagnostics_,
                                            FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
                                            TimeStampStatusParam()));

      // open zvision lidar input device or file
      if (dump_file != "")  // have PCAP file?
      {
          // read data from packet capture file
          input_.reset(new zvision_lidar_driver::InputPCAP(private_nh, port_to_recv_udppkt, packet_rate_MLXS, dump_file));
      }
      else
      {
          // read data from live socket
          input_.reset(new zvision_lidar_driver::InputSocket(private_nh, port_to_recv_udppkt));
      }
      // raw packet output topic
      output_ = node.advertise<zvision_lidar_msgs::zvisionLidarScan>("zvision_lidar_packets", 20);
  }
  else{
  	ROS_ERROR_STREAM("unknown LIDAR model!");
  }
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool zvisionLidarDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  zvision_lidar_msgs::zvisionLidarScanPtr scan(new zvision_lidar_msgs::zvisionLidarScan);

  scan->packets.resize(config_.npackets);
  for (int i = 0; i < config_.npackets; ++i)
  {
    while (true)
    {
#if 0
      // keep reading until full packet received
      int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
      if (rc == 0)
        break;  // got a full packet?
      if (rc < 0)
        return false;  // end of file reached?
#else
	  // keep reading until full packet received
	  int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
	  int udp_seq = 0;

	  if (rc < 0)
		  return false;  // end of file reached?

	  if (rc == 0)
	  {
          udp_seq = (int)(scan->packets[i].data[3]) + ((scan->packets[i].data[2] & 0xF) << 8);
	  }

	  if (i == udp_seq)
	  {
		  break;
	  }
	  else
	  {
		  //ROS_INFO("Find a packet, seq wanted[%d], seq read[%d]", i, udp_seq);
	  }
#endif
    }
  }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full zvision lidar scan.");
  scan->header.stamp = scan->packets.back().stamp;
  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);
 
  // notify diagnostics that a message has been published, updating its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  return true;
}

void zvisionLidarDriver::callback(zvision_lidar_driver::zvisionLidarNodeConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
}
}
