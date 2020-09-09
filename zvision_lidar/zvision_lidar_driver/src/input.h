/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *	Copyright (C) 2017, Robosense, Tony Zhang
 *  Copyright (C) 2019, Zvision, Pengfei Cui
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the ZVISION LIDAR 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#ifndef __ZVISION_LIDAR_INPUT_H_
#define __ZVISION_LIDAR_INPUT_H_

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <zvision_lidar_msgs/zvisionLidarPacket.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>

namespace zvision_lidar_driver
{
static uint16_t UDP_DATA_PORT_NUMBER = 2368;   // zvision lidar default data port on PC??
                                                /**
                                                 *  get udp pkt（from *.pcap or ethernet )
                                                 * @brief The Input class,
                                                     *
                                                     * @param private_nh  NodeHandled
                                                     * @param port
                                                     * @returns 0 if successful,
                                                     *          -1 if end of file
                                                     *          >0 if incomplete packet (is this possible?)
                                                 */
class Input
{
public:
  Input(ros::NodeHandle private_nh, uint16_t port);

  virtual ~Input()
  {
  }

  virtual int getPacket(zvision_lidar_msgs::zvisionLidarPacket* pkt, const double time_offset) = 0;

protected:
  ros::NodeHandle private_nh_;
  uint16_t port_;
  std::string devip_str_;
};

/** @brief Live zvision lidar input from socket. */
class InputSocket : public Input
{
public:
  InputSocket(ros::NodeHandle private_nh, uint16_t port = UDP_DATA_PORT_NUMBER);

  virtual ~InputSocket();

  virtual int getPacket(zvision_lidar_msgs::zvisionLidarPacket* pkt, const double time_offset);

private:
  int sockfd_;
  in_addr devip_;//Only accepting packets from IP address

  int Ret;
  int len;
};

/** @brief zvision lidar input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap
   */
class InputPCAP : public Input
{
public:
  InputPCAP(ros::NodeHandle private_nh, uint16_t port = UDP_DATA_PORT_NUMBER, double packet_rate = 0.0,
            std::string filename = "", bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);

  virtual ~InputPCAP();

  virtual int getPacket(zvision_lidar_msgs::zvisionLidarPacket* pkt, const double time_offset);

private:
  ros::Rate packet_rate_;
  std::string filename_;
  pcap_t* pcap_;
  bpf_program pcap_packet_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_;
  bool read_once_;
  bool read_fast_;
  double repeat_delay_;
};
}

#endif  // __ZVISION_LIDAR_INPUT_H
