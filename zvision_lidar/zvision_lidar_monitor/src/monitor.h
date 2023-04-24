#pragma once

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <ros/package.h>
#include<zvision_lidar_msgs/zvisionLidarHeartbeat.h>
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
static uint16_t UDP_HEARTBEAT_PORT_NUMBER = 55000;
class zvisionLidarMonitor
{
public:
  /**
 * @brief zvisionLidarDriver
 * @param node          raw packet output topic
 * @param private_nh    nodehandler
 */
  zvisionLidarMonitor(ros::NodeHandle node, ros::NodeHandle private_nh);

  virtual ~zvisionLidarMonitor();

  bool pollMsg(void);

private:

  struct
  {
    std::string model;
    std::string ip;
    int port;
    bool pub_msg;
  } config_;

int initSocket();

int getPacket(zvision_lidar_msgs::zvisionLidarHeartbeatPtr& ht);

int parsingPacket(zvision_lidar_msgs::zvisionLidarHeartbeatPtr& ht);

private:
  int sockfd_;
  in_addr devip_;
  ros::Publisher output_;

};

}
