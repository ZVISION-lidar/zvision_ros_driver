/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  Copyright (C) 2019 Zvision Technology, Pengfei Cui
 *  Copyright (C) 2021, Zvision, Senke Chen
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the ZVISION LIDAR 3D LIDARs
 */
#include "zvision_driver.h"
#include <zvision_lidar_msgs/msg/zvision_lidar_scan.h>

namespace zvision_lidar_driver
{
zvisionLidarDriver::zvisionLidarDriver(const rclcpp::NodeOptions & options)
        : rclcpp::Node("zvision_driver_node", options),
          diagnostics_(this, 0.2)
{

  // use private node handle to get parameters
    config_.frame_id = this->declare_parameter("frame_id", std::string("zvision_lidar"));
//  private_nh.param("frame_id", config_.frame_id, std::string("zvision_lidar"));
    config_.model = this->declare_parameter("model", std::string("ML30SA1"));
//    private_nh.param("model", config_.model, std::string("ML30SA1"));
// TODO: tf
//   std::string tf_prefix = tf::getPrefixParam(private_nh);
//   RCLCPP_DEBUG_STREAM(this->get_logger(),"tf_prefix: " << tf_prefix);
//   config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);


  int fre = 10;
  fre = this->declare_parameter("frequency", 10);
//  private_nh.param("frequency", fre, 10);
  double packet_rate_ML30 = 125 * fre;
  double packet_rate_ML30S_A1 = 160 * fre;
  double packet_rate_MLX = 400 * fre;
  double packet_rate_MLXA1 = 475 * fre;
  double packet_rate_MLXS = 450 * fre;
  std::string model_full_name;
  future_ = exit_signal_.get_future();

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
    RCLCPP_ERROR_STREAM(this->get_logger(),"unknown LIDAR model: " << config_.model);
  }
  std::string deviceName(std::string("Zvision ") + model_full_name);


  if(config_.model == "ML30B1"){
	  int npackets = 125;
      config_.npackets = this->declare_parameter("npackets", npackets);
//	  private_nh.param("npackets", config_.npackets, npackets);
	  RCLCPP_INFO_STREAM(this->get_logger(),"publishing " << config_.npackets << " packets per scan");

	  std::string dump_file;
      dump_file = this->declare_parameter("pcap", std::string(""));
//	  private_nh.param("pcap", dump_file, std::string(""));

	  int port_to_recv_udppkt;
      port_to_recv_udppkt = this->declare_parameter("udp_port", (int)UDP_DATA_PORT_NUMBER);
//	  private_nh.param("udp_port", port_to_recv_udppkt, (int)UDP_DATA_PORT_NUMBER);

	  // Initialize dynamic reconfigure
//	  srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> >(private_nh);
//	  dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig>::CallbackType f;
//	  f = boost::bind(&zvisionLidarDriver::callback, this, _1, _2);
//	  srv_->setCallback(f);  // Set callback function und call initially

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
		  input_.reset(new zvision_lidar_driver::InputPCAP(this, port_to_recv_udppkt, packet_rate_ML30, dump_file));
	  }
	  else
	  {
		  // read data from live socket
		  input_.reset(new zvision_lidar_driver::InputSocket(this, port_to_recv_udppkt));
	  }
	  // raw packet output topic
//	  output_ = node.advertise<zvision_lidar_msgs::msg::ZvisionLidarScan>("zvision_lidar_packets", 20);
      output_ =
              this->create_publisher<zvision_lidar_msgs::msg::ZvisionLidarScan>("zvision_lidar_packets", 20);
  }
  else if(config_.model == "ML30SA1"){

	  int npackets = 160;
      config_.npackets = this->declare_parameter("npackets", npackets);
//	  private_nh.param("npackets", config_.npackets, npackets);
      RCLCPP_INFO_STREAM(this->get_logger(),"publishing " << config_.npackets << " packets per scan");

      std::string dump_file;
      dump_file = this->declare_parameter("pcap", std::string(""));
//	  private_nh.param("pcap", dump_file, std::string(""));

      int port_to_recv_udppkt;
      port_to_recv_udppkt = this->declare_parameter("udp_port", (int)UDP_DATA_PORT_NUMBER);
//	  private_nh.param("udp_port", port_to_recv_udppkt, (int)UDP_DATA_PORT_NUMBER);

	  // Initialize dynamic reconfigure
//	  srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> >(private_nh);
//	  dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig>::CallbackType f;
//	  f = boost::bind(&zvisionLidarDriver::callback, this, _1, _2);
//	  srv_->setCallback(f);  // Set callback function und call initially

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
		  input_.reset(new zvision_lidar_driver::InputPCAP(this, port_to_recv_udppkt, packet_rate_ML30S_A1, dump_file));
	  }
	  else
	  {
		  // read data from live socket
		  input_.reset(new zvision_lidar_driver::InputSocket(this, port_to_recv_udppkt));
	  }
	  // raw packet output topic
//	  output_ = node.advertise<zvision_lidar_msgs::msg::ZvisionLidarScan>("zvision_lidar_packets", 20);
      output_ =
              this->create_publisher<zvision_lidar_msgs::msg::ZvisionLidarScan>("zvision_lidar_packets", 20);
  }
  else if(config_.model == "MLX"){
      int npackets = 400;
      config_.npackets = this->declare_parameter("npackets", npackets);
//	  private_nh.param("npackets", config_.npackets, npackets);
      RCLCPP_INFO_STREAM(this->get_logger(),"publishing " << config_.npackets << " packets per scan");

      std::string dump_file;
      dump_file = this->declare_parameter("pcap", std::string(""));
//	  private_nh.param("pcap", dump_file, std::string(""));

      int port_to_recv_udppkt;
      port_to_recv_udppkt = this->declare_parameter("udp_port", (int)UDP_DATA_PORT_NUMBER);
//	  private_nh.param("udp_port", port_to_recv_udppkt, (int)UDP_DATA_PORT_NUMBER);

      // Initialize dynamic reconfigure
//      srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> >(private_nh);
//      dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig>::CallbackType f;
//      f = boost::bind(&zvisionLidarDriver::callback, this, _1, _2);
//      srv_->setCallback(f);  // Set callback function und call initially

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
          input_.reset(new zvision_lidar_driver::InputPCAP(this, port_to_recv_udppkt, packet_rate_MLX, dump_file));
      }
      else
      {
          // read data from live socket
          input_.reset(new zvision_lidar_driver::InputSocket(this, port_to_recv_udppkt));
      }
      // raw packet output topic
//      output_ = node.advertise<zvision_lidar_msgs::msg::ZvisionLidarScan>("zvision_lidar_packets", 20);
      output_ =
              this->create_publisher<zvision_lidar_msgs::msg::ZvisionLidarScan>("zvision_lidar_packets", 20);
  }
  else if(config_.model == "MLXA1"){
      int npackets = 475;
      config_.npackets = this->declare_parameter("npackets", npackets);
//	  private_nh.param("npackets", config_.npackets, npackets);
      RCLCPP_INFO_STREAM(this->get_logger(),"publishing " << config_.npackets << " packets per scan");

      std::string dump_file;
      dump_file = this->declare_parameter("pcap", std::string(""));
//	  private_nh.param("pcap", dump_file, std::string(""));

      int port_to_recv_udppkt;
      port_to_recv_udppkt = this->declare_parameter("udp_port", (int)UDP_DATA_PORT_NUMBER);
//	  private_nh.param("udp_port", port_to_recv_udppkt, (int)UDP_DATA_PORT_NUMBER);

      // Initialize dynamic reconfigure
//      srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> >(private_nh);
//      dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig>::CallbackType f;
//      f = boost::bind(&zvisionLidarDriver::callback, this, _1, _2);
//      srv_->setCallback(f);  // Set callback function und call initially

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
          input_.reset(new zvision_lidar_driver::InputPCAP(this, port_to_recv_udppkt, packet_rate_MLXA1, dump_file));
      }
      else
      {
          // read data from live socket
          input_.reset(new zvision_lidar_driver::InputSocket(this, port_to_recv_udppkt));
      }
      // raw packet output topic
//      output_ = node.advertise<zvision_lidar_msgs::msg::ZvisionLidarScan>("zvision_lidar_packets", 20);
      output_ =
              this->create_publisher<zvision_lidar_msgs::msg::ZvisionLidarScan>("zvision_lidar_packets", 20);
  }
  else if(config_.model == "MLXS"){
      int npackets = 450;
      config_.npackets = this->declare_parameter("npackets", npackets);
//	  private_nh.param("npackets", config_.npackets, npackets);
      RCLCPP_INFO_STREAM(this->get_logger(),"publishing " << config_.npackets << " packets per scan");

      std::string dump_file;
      dump_file = this->declare_parameter("pcap", std::string(""));
//	  private_nh.param("pcap", dump_file, std::string(""));

      int port_to_recv_udppkt;
      port_to_recv_udppkt = this->declare_parameter("udp_port", (int)UDP_DATA_PORT_NUMBER);
//	  private_nh.param("udp_port", port_to_recv_udppkt, (int)UDP_DATA_PORT_NUMBER);

      // Initialize dynamic reconfigure
//      srv_ = boost::make_shared<dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig> >(private_nh);
//      dynamic_reconfigure::Server<zvision_lidar_driver::zvisionLidarNodeConfig>::CallbackType f;
//      f = boost::bind(&zvisionLidarDriver::callback, this, _1, _2);
//      srv_->setCallback(f);  // Set callback function und call initially

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
          input_.reset(new zvision_lidar_driver::InputPCAP(this, port_to_recv_udppkt, packet_rate_MLXS, dump_file));
      }
      else
      {
          // read data from live socket
          input_.reset(new zvision_lidar_driver::InputSocket(this, port_to_recv_udppkt));
      }
      // raw packet output topic
//      output_ = node.advertise<zvision_lidar_msgs::msg::ZvisionLidarScan>("zvision_lidar_packets", 20);
      output_ =
              this->create_publisher<zvision_lidar_msgs::msg::ZvisionLidarScan>("zvision_lidar_packets", 20);
  }
  else{
  	RCLCPP_ERROR_STREAM(this->get_logger(),"unknown LIDAR model!");
  }
    poll_thread_ = std::thread(&zvisionLidarDriver::pollThread, this);

}

zvisionLidarDriver::~zvisionLidarDriver()
{
  exit_signal_.set_value();
  poll_thread_.join();
}
/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool zvisionLidarDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
//  zvision_lidar_msgs::msg::ZvisionLidarScanPtr scan(new zvision_lidar_msgs::msg::ZvisionLidarScan);
    std::unique_ptr<zvision_lidar_msgs::msg::ZvisionLidarScan> scan =
            std::make_unique<zvision_lidar_msgs::msg::ZvisionLidarScan>();
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
      //   RCLCPP_INFO(this->get_logger(),"seq read[%d]", udp_seq);

	  if (i == udp_seq)
	  {
		  break;
	  }
	  else
	  {
		//   RCLCPP_INFO(this->get_logger(),"Find a packet, seq wanted[%d], seq read[%d]", i, udp_seq);
	  }
#endif
    }
  }

  // publish message using time of last packet read
  RCLCPP_DEBUG(this->get_logger(),"Publishing a full zvision lidar scan.");
  builtin_interfaces::msg::Time stamp = scan->packets.back().stamp;
  scan->header.stamp = stamp;
  scan->header.frame_id = config_.frame_id;
  output_->publish(std::move(scan));

//        output_.publish(scan);
 
  // notify diagnostics that a message has been published, updating its status

  diag_topic_->tick(stamp);
//   diagnostics_.update();

  return true;
}
void zvisionLidarDriver::pollThread()
{
  std::future_status status;

  do {
    poll();
    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}
// void zvisionLidarDriver::callback(zvision_lidar_driver::zvisionLidarNodeConfig& config, uint32_t level)
// {
//   RCLCPP_INFO(this->get_logger(),"Reconfigure Request");
//   config_.time_offset = config.time_offset;
// }
}
RCLCPP_COMPONENTS_REGISTER_NODE(zvision_lidar_driver::zvisionLidarDriver)
