/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *	Copyright (C) 2017, Robosense, Tony Zhang
 *	Copyright (C) 2019, Zvision, Pengfei Cui
 *  Copyright (C) 2021, Zvision, Senke Chen
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the ZVISION LIDAR ML30 3D LIDAR:
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
#include "input.h"

extern volatile sig_atomic_t flag;
namespace zvision_lidar_driver
{
static const size_t packet_size = sizeof(zvision_lidar_msgs::msg::ZvisionLidarPacket().data);

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */



Input::Input(rclcpp::Node * private_nh, uint16_t port) : private_nh_(private_nh), port_(port)
{
    devip_str_ = private_nh->declare_parameter("device_ip",std::string(""));
    if (!devip_str_.empty())
        RCLCPP_INFO_STREAM(private_nh->get_logger(),"Only accepting packets from IP address: " << devip_str_);
}

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
InputSocket::InputSocket(rclcpp::Node * private_nh, uint16_t port) : Input(private_nh, port)
{
  sockfd_ = -1;

  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }

  RCLCPP_INFO_STREAM(private_nh->get_logger(),"Opening UDP socket: port " << port);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    perror("socket");  // TODO: RCLCPP_ERROR errno
    return;
  }

  int opt = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
  {
    perror("setsockopt error!\n");
    return;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
    perror("bind");  // TODO: RCLCPP_ERROR errno
    return;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }
}

/** @brief destructor */
InputSocket::~InputSocket(void)
{
  (void)close(sockfd_);
}

/** @brief Get one zvision lidar packet. */
int InputSocket::getPacket(zvision_lidar_msgs::msg::ZvisionLidarPacket* pkt, const double time_offset)
{
  rclcpp::Time time1 = private_nh_->get_clock()->now();
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
  while (flag == 1)
  {
    // Receive packets that should now be available from the
    // socket using a blocking read.
    // poll() until input available
    do
    {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0)  // poll() error?
      {
        if (errno != EINTR)
          RCLCPP_ERROR(private_nh_->get_logger(),"poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0)  // poll() timeout?
      {
        RCLCPP_WARN(private_nh_->get_logger(),"zvision lidar poll() timeout");

        char buffer_data[8] = "re-con";
        memset(&sender_address, 0, sender_address_len);          // initialize to zeros
        sender_address.sin_family = AF_INET;                     // host byte order
        sender_address.sin_port = htons(UDP_DATA_PORT_NUMBER);  // port in network byte order, set any value
        sender_address.sin_addr.s_addr = devip_.s_addr;          // automatically fill in my IP
        sendto(sockfd_, &buffer_data, strlen(buffer_data), 0, (sockaddr*)&sender_address, sender_address_len);
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
      {
        RCLCPP_ERROR(private_nh_->get_logger(),"poll() reports zvision lidar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);
    ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], packet_size, 0, (sockaddr*)&sender_address, &sender_address_len);

    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        perror("recvfail");
        RCLCPP_INFO(private_nh_->get_logger(),"recvfail");
        return 1;
      }
    }
    else if ((size_t)nbytes == packet_size)
    {
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
        continue;
      else
        break;  // done
    }

    RCLCPP_DEBUG_STREAM(private_nh_->get_logger(),"incomplete zvision lidar packet read: " << nbytes << " bytes");
  }
  if (flag == 0)
  {
    abort();
  }
  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  rclcpp::Time time2 = private_nh_->get_clock()->now();
  pkt->stamp =  rclcpp::Time((time2.nanoseconds() + time1.nanoseconds()) / 2.0 + time_offset);

  return 0;
}

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */

InputPCAP::InputPCAP(rclcpp::Node * private_nh, uint16_t port, double packet_rate, std::string filename,
                     bool read_once, bool read_fast, double repeat_delay)
  : Input(private_nh, port), packet_rate_(packet_rate), filename_(filename)
{
  pcap_ = NULL;
  empty_ = true;

  // get parameters using private node handle
  read_once_ = private_nh->declare_parameter("read_once",false);
  read_fast_ = private_nh->declare_parameter("read_fast",false);
  repeat_delay_ = private_nh->declare_parameter("repeat_delay",0.0);
  if (read_once_)
    RCLCPP_INFO(private_nh->get_logger(),"Read input file only once.");
  if (read_fast_)
    RCLCPP_INFO(private_nh->get_logger(),"Read input file as quickly as possible.");
  if (repeat_delay_ > 0.0)
    RCLCPP_INFO(private_nh->get_logger(),"Delay %.3f seconds before repeating input file.", repeat_delay_);
  // Open the PCAP dump file
  RCLCPP_INFO_STREAM(private_nh->get_logger(),"Opening PCAP file " << filename_);
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
  {
    RCLCPP_FATAL(private_nh->get_logger(),"Error opening zvision lidar socket dump file.");
    return;
  }
  std::stringstream filter;
  if (devip_str_ != "")  // using specific IP?
  {
    filter << "src host " << devip_str_ << " && ";
  }
  filter << "udp dst port " << port;
  pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
}

/** destructor */
InputPCAP::~InputPCAP(void)
{
  pcap_close(pcap_);
}

/** @brief Get one zvision lidar packet. */
int InputPCAP::getPacket(zvision_lidar_msgs::msg::ZvisionLidarPacket* pkt, const double time_offset)
{
  struct pcap_pkthdr* header;
  const u_char* pkt_data;

  while (flag == 1)
  {
    int res;
    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
    {
      // Skip packets not for the correct port and from the
      // selected IP address.
      if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
        continue;

      // Keep the reader from blowing through the file.
      if (read_fast_ == false)
        packet_rate_.sleep();

      memcpy(&pkt->data[0], pkt_data + 42, packet_size);/*42 bytes udp header*/
      pkt->stamp = private_nh_->get_clock()->now();  // time_offset not considered here, as no
                                      // synchronization required


      empty_ = false;
      return 0;  // success
    }

    if (empty_)  // no data in file?
    {
      RCLCPP_WARN(private_nh_->get_logger(),"Error %d reading zvision lidar packet: %s", res, pcap_geterr(pcap_));
      return -1;
    }

    if (read_once_)
    {
      RCLCPP_INFO(private_nh_->get_logger(),"end of file reached -- done reading.");
      return -1;
    }

    if (repeat_delay_ > 0.0)
    {
      RCLCPP_INFO(private_nh_->get_logger(),"end of file reached -- delaying %.3f seconds.", repeat_delay_);
      usleep(rint(repeat_delay_ * 1000000.0));
    }

    RCLCPP_DEBUG(private_nh_->get_logger(),"replaying zvision lidar dump file");

    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;  // maybe the file disappeared?
  }                 // loop back and try again

  if (flag == 0)
  {
    abort();//termination
  }
}
}
