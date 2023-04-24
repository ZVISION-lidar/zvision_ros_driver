#include "monitor.h"
#include <tf/transform_listener.h>

extern volatile sig_atomic_t flag;
namespace zvision_lidar_driver
{

static const size_t heartbeat_packet_size = sizeof(zvision_lidar_msgs::zvisionLidarHeartbeat().data);

  void ResolveIp2String(const unsigned char* addr, std::string& ip)
  {
       char cip[128] = "";
      sprintf(cip, "%u.%u.%u.%u", addr[0], addr[1], addr[2], addr[3]);
      ip = std::string(cip);
  }

  void ResolveMac2String(const unsigned char* addr, std::string& ip)
  {
       char cip[128] = "";
      sprintf(cip, "%02X-%02X-%02X-%02X-%02X-%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
      ip = std::string(cip);
  }




zvisionLidarMonitor::zvisionLidarMonitor(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  //get parameters
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);

  private_nh.param("model", config_.model, std::string("ML30SB2"));
  private_nh.param("device_ip", config_.ip, std::string("INADDR_ANY"));
  devip_.s_addr = INADDR_ANY;
  std::string ip_str = "INADDR_ANY";
  // in_addr_t addr = inet_addr(config_.ip.c_str());
  // if( addr != INADDR_NONE ){
  //   ip_str = config_.ip;
  //   devip_.s_addr = addr;
  // }

  private_nh.param("udp_port", config_.port, (int)UDP_HEARTBEAT_PORT_NUMBER);
  private_nh.param("publish_msg", config_.pub_msg, true);
  ROS_INFO_STREAM("Opening UDP socket:  ip: " <<  ip_str.c_str()
                                                                                    <<",  port: "<<config_.port);
  // model
  std::string model_full_name;
  if (config_.model == "ML30SB2")
  {
    model_full_name = "ZVISION-LiDAR-ML30SB2";
    output_ = node.advertise<zvision_lidar_msgs::zvisionLidarHeartbeat>("zvision_lidar_heartbeat", 20);
  }
  else
  {
    ROS_ERROR_STREAM("unknown LIDAR model: " << config_.model);
    return;
  }

  ROS_INFO("Zvision LIDAR Model [ %s ]: %s", config_.model.c_str(), model_full_name.c_str());

  // init socket
   initSocket(); 

}

zvisionLidarMonitor::~zvisionLidarMonitor(void)
{
      (void)close(sockfd_);
}

int zvisionLidarMonitor::initSocket(){
    if(sockfd_ != -1)
        (void)close(sockfd_);
    
  sockfd_ = -1;
  
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    perror("socket"); 
    return -1;
  }

  int opt = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
  {
    perror("setsockopt error!\n");
    sockfd_ = -1;
    return -1;
  }

  sockaddr_in my_addr;
  memset(&my_addr, 0, sizeof(my_addr));
  my_addr.sin_family = AF_INET; 
  my_addr.sin_port = htons(config_.port); 
  my_addr.sin_addr.s_addr = devip_.s_addr;

  if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
    perror("bind"); 
    sockfd_ = -1;
    return -1;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    sockfd_ = -1;
    return -1;
  }

  ROS_INFO("init monitor socket done!");
  return 0;
}

/** get heartbeat packet
 * 
 *  @returns true while received packet within timeout
*/
int zvisionLidarMonitor::getPacket(zvision_lidar_msgs::zvisionLidarHeartbeatPtr& ht){

  if(sockfd_ == -1){
    ROS_ERROR("monitor initialization failed");
    ros::Duration(1).sleep();
    return -1;
  }
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 3000; 

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
  while (flag == 1)
  {
    // pre
    do
    {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0)
      {
        if (errno != EINTR)
          ROS_ERROR("poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0)
      {
        ROS_WARN("zvision lidar poll() timeout");
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
      {
        ROS_ERROR("poll() reports zvision lidar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);

    // receive
    ssize_t nbytes = recvfrom(sockfd_, &ht->data[0], heartbeat_packet_size, 0, (sockaddr*)&sender_address, &sender_address_len);

    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        perror("recvfail");
        ROS_INFO("recvfail");
        return 1;
      }
    }

    ht->data_len = nbytes;
    if (devip_.s_addr != INADDR_ANY  && sender_address.sin_addr.s_addr != devip_.s_addr)
        continue;
    else
      break;

    ROS_DEBUG_STREAM("incomplete zvision lidar packet read: " << nbytes << " bytes");
  }
  if (flag == 0)
  {
    abort();
  }

  return 0;
}

int zvisionLidarMonitor::parsingPacket(zvision_lidar_msgs::zvisionLidarHeartbeatPtr& ht){

   int offset = 0;
  std::string pkt((char*)(&ht->data[0]), ht->data_len);
  //const char* pdata = pkt.c_str();
  uint8_t* pdata = &(ht->data[0]);
   if(ht->data_len == 512){


   }
   // parsing ML30SB2
   else if(ht->data_len == 138){
    std::string info;
    std::string tmp_str;
    // header
    ht->tag = pkt.substr(0,12);
    info += "tag: "+  ht->tag +"\n";
    offset = 12;

    memcpy(&ht->version[0], pdata + offset, 2);
    offset += 2;
    {
      char cip[128] = "";
      sprintf(cip, "%u.%u", ht->version[0], ht->version[1]);
      info += "version: " +  std::string(cip) + "\n";
    }
    
    // mtd_info
    ht->sn = pkt.substr(offset,18);
    info += "sn: "+  ht->sn +"\n";
    offset += 18;

    memcpy(&ht->fpga_version[0], pdata + offset, 4);
    offset += 4;
    ResolveIp2String(&ht->fpga_version[0], tmp_str);
    info += "fpga version: "+  tmp_str +"\n";

    memcpy(&ht->fpga_file_size, pdata + offset, 4);
    offset += 4;
    ht->fpga_file_size = ntohl(ht->fpga_file_size);
    info += "fpga file size: "+  std::to_string(ht->fpga_file_size) +"\n";

    memcpy(&ht->env_version[0], pdata + offset, 4);
    offset += 4;
    ResolveIp2String(&ht->env_version[0], tmp_str);
    info += "env version: "+  tmp_str +"\n";

    memcpy(&ht->env_file_size, pdata + offset, 4);
    offset += 4;
    ht->env_file_size = ntohl(ht->env_file_size);
    info += "env file size: "+  std::to_string(ht->env_file_size) +"\n";

    memcpy(&ht->embadded_version[0], pdata + offset, 4);
    offset += 4;
    ResolveIp2String(&ht->embadded_version[0], tmp_str);
    info += "embadded version: "+  tmp_str +"\n";

    memcpy(&ht->embadded_file_size, pdata + offset, 4);
    offset += 4;
    ht->embadded_file_size = ntohl(ht->embadded_file_size);
    info += "embadded file size: "+  std::to_string(ht->embadded_file_size) +"\n";

    ht->sys_diag_status = *(pdata + offset);
    offset++;
    std::vector<std::string> diag_sys_strs = {"Low temperature start", "Over temperature", 
                                                                                "Hardware failure", "Software failure", 
                                                                                "Supply voltage fault", "PointCloud loss fault",
                                                                                 "", ""};
    {
      std::string msg;
      uint8_t flg = 0x01;
      for(int i = 0;i<8;i++){
        uint8_t bit_i = flg<<i;
        if((bit_i & ht->sys_diag_status)  == bit_i){
          msg += diag_sys_strs[i] + ", ";
        }
        if(i >= 5)
          break;
      }
      if(msg.empty())
        msg = "Normal";
      else
        msg = msg.substr(0, msg.size() - 2);
      
      info += "sys diag status: "+  msg +"\n";
    }

    std::vector<std::string> diag_hw_strs = {"ADC0 CLK", "ADC1 CLK","up fov start ghost state","down fov start ghost state"
                                                                                , "i2c", "mems drive voltage", "mems high voltage", "apd bias voltage"
                                                                                , "pac19xx state", "5.5v power", "5.5v voltage", "1.8v voltage"
                                                                                , "3.3v voltage", "RB3.3v voltage", "RB5.0v voltage", "LD12v voltage"
                                                                                , "", "", "", ""
                                                                                , "", "", "", ""
                                                                                , "", "", "", ""
                                                                                , "", "", "", ""};
    memcpy(&ht->hardware_diag_status, pdata + offset, 4);
    offset += 4;
    ht->hardware_diag_status = ntohl(ht->hardware_diag_status);
    {
      std::string msg;
      uint32_t flg = 0x01;
      for(int i = 0;i<32;i++){
        uint32_t bit_i = flg<<i;
          if((bit_i & ht->hardware_diag_status)  == bit_i){
              msg += diag_hw_strs[i] + " error, ";
          }
          if(i >= 15)
              break;
      }
      if(msg.empty())
        msg = "Normal";
      else
        msg = msg.substr(0, msg.size() - 2);
      
      info += "hardware diag status: "+  msg +"\n";
    }

    ht->ptp_sync_status = *(pdata + offset);
    offset++;
    {
      info += "ptp sync status: ";
      if((ht->ptp_sync_status & 0x01) == 0x01)
        info += "lock";
      else
        info += "unlock";
      info +="\n";
      info += "dirty check: ";
      if((ht->ptp_sync_status & 0x10) == 0x10)
        info += "dirty";
      else
        info += "clear";
      info +="\n";
    }
    // reserved 2 bytes
    offset +=2;

    memcpy(&ht->ip[0], pdata + offset, 4);
    offset += 4;
    ResolveIp2String(&ht->ip[0], tmp_str);
    info += "ip: "+  tmp_str +"\n";

    memcpy(&ht->port, pdata + offset, 4);
    offset += 4;
    ht->port = ntohl(ht->port);
    info += "port: "+  std::to_string(ht->port) +"\n";

    ht->time_sync_mode = *(pdata + offset);
    offset++;
    info += "time sync mode: ";
    if(ht->time_sync_mode == 0x01 || ht->time_sync_mode == 0xFF){
      info += "PTP";
    }else if(ht->time_sync_mode == 0x02){
        info += "GPS";
    }else{
      info += "Unknown";
    }
    info += "\n";

    memcpy(&ht->dst_ip[0], pdata + offset, 4);
    offset += 4;
    ResolveIp2String(&ht->dst_ip[0], tmp_str);
    info += "dst ip: "+  tmp_str +"\n";

    ht->retro_switch = *(pdata + offset);
    offset++;
    info += "retro switch: ";
    if(ht->retro_switch == 0x00 || ht->retro_switch == 0xFF){
      info += "Off";
    }else if(ht->retro_switch == 0x01){
      info += "On";
    }else{
      info += "Unknown";
    }
    info += "\n";

    memcpy(&ht->net_mask[0], pdata + offset, 4);
    offset += 4;
    ResolveIp2String(&ht->net_mask[0], tmp_str);
    info += "net mask: "+  tmp_str +"\n";

    memcpy(&ht->cfg_mac_addr[0], pdata + offset, 6);
    offset += 6;
    ResolveMac2String(&ht->cfg_mac_addr[0], tmp_str);
    info += "cfg mac addr: "+  tmp_str +"\n";

    memcpy(&ht->frame_sync_offset, pdata + offset, 4);
    offset += 4;
    ht->frame_sync_offset = ntohl(ht->frame_sync_offset);
    info += "frame sync offset(5ns): "+  std::to_string(ht->frame_sync_offset) +"\n";

    ht->echo_mode = *(pdata + offset);
    offset++;
    info += "echo mode: ";
    if (0x01 ==  ht->echo_mode || 0xFF ==  ht->echo_mode)
      info += "EchoSingleFirst";
    else if (0x02 ==  ht->echo_mode)
      info += "EchoSingleStrongest";
     else if (0x04 ==  ht->echo_mode)
       info += "EchoSingleLast";
     else if (0x03 ==  ht->echo_mode)
       info += "EchoDoubleFirstStrongest";
     else if (0x05 ==  ht->echo_mode)
       info += "EchoDoubleFirstLast";
     else if (0x06 ==  ht->echo_mode)
       info+= "EchoDoubleStrongestLast";
     else
        info+= "Unknown";
      info += "\n";

    ht->frame_sync_switch = *(pdata + offset);
    offset++;
    info += "frame sync switch: ";
    if(ht->frame_sync_switch == 0x00 || ht->frame_sync_switch == 0xFF){
      info += "Off";
    }else if(ht->frame_sync_switch == 0x01){
      info += "On";
    }else{
      info += "Unknown";
    }
    info += "\n";

    memcpy(&ht->retro_intensity_percentage[0], pdata + offset, 2);
    offset += 2;
    info += "retro intensity percentage: " + std::to_string(ht->retro_intensity_percentage[0])
                                                                      +", " + std::to_string(ht->retro_intensity_percentage[1])
                                                                      +"\n";

    ht->angle_send_switch = *(pdata + offset);
    offset++;
    info += "angle send switch: ";
    if(ht->angle_send_switch == 0x00 || ht->angle_send_switch == 0xFF){
      info += "Off";
    }else if(ht->angle_send_switch == 0x01){
      info += "On";
    }else{
      info += "Unknown";
    }
    info += "\n";

    ht->downsample_mode = *(pdata + offset);
    offset++;
    info += "downsample mode: ";
    if(ht->downsample_mode == 0x00 || ht->downsample_mode == 0xFF){
      info += "None";
    }else if(ht->downsample_mode == 0x01){
      info += "1/2";
    }else if(ht->downsample_mode == 0x02){
      info += "1/4";
    }else{
      info += "Unknown";
    }
    info += "\n";

    memcpy(&ht->dirty_check_thres_set_reset[0], pdata + offset, 2);
    ht->dirty_check_thres_set_reset[0] = ntohs(ht->dirty_check_thres_set_reset[0]);
    offset += 2;
    memcpy(&ht->dirty_check_thres_set_reset[1], pdata + offset, 2);
    ht->dirty_check_thres_set_reset[1] = ntohs(ht->dirty_check_thres_set_reset[1]);
    offset += 2;
    info += "dirty check thres (set, reset): " + std::to_string(ht->dirty_check_thres_set_reset[0])
                                                                         + ", " + std::to_string(ht->dirty_check_thres_set_reset[1])
                                                                                    +"\n";

    ht->dirty_switch = *(pdata + offset);
    offset++;
    info += "dirty switch: ";
    if(ht->dirty_switch == 0x00 || ht->dirty_switch == 0xFF){
      info += "Off";
    }else if(ht->dirty_switch == 0x01){
      info += "On";
    }else{
      info += "Unknown";
    }
    info += "\n";

    ht->dirty_fresh_switch = *(pdata + offset);
    offset++;
    info += "dirty fresh switch: ";
    if(ht->dirty_fresh_switch == 0x00 || ht->dirty_fresh_switch == 0xFF){
      info += "Off";
    }else if(ht->dirty_fresh_switch == 0x01){
      info += "On";
    }else{
      info += "Unknown";
    }
    info += "\n";


    memcpy(&ht->dirty_detect_cycle, pdata + offset, 2);
    offset += 2;
    ht->dirty_detect_cycle = ntohs(ht->dirty_detect_cycle);
    info += "dirty detect cycle: " + std::to_string(ht->dirty_detect_cycle) + "\n";

    ht->diag_switch = *(pdata + offset);
    offset++;
    info += "diag switch: ";
    if(ht->diag_switch == 0x00 || ht->diag_switch == 0xFF){
      info += "Off";
    }else if(ht->diag_switch == 0x01){
      info += "On";
    }else{
      info += "Unknown";
    }
    info += "\n";

    memcpy(&ht->diag_inner_thres, pdata + offset, 2);
    offset += 2;
    ht->diag_inner_thres = ntohs(ht->diag_inner_thres);
    info += "diag inner thres: " + std::to_string(ht->diag_inner_thres) + "\n";

    memcpy(&ht->diag_outer_thres, pdata + offset, 2);
    offset += 2;
    ht->diag_outer_thres = ntohs(ht->diag_outer_thres);
    info += "diag outer thres: " + std::to_string(ht->diag_outer_thres) + "\n";

    memcpy(&ht->point_loss_thres, pdata + offset, 2);
    offset += 2;
    ht->point_loss_thres = ntohs(ht->point_loss_thres);
    info += "point loss thres: " + std::to_string(ht->point_loss_thres) + "\n";

    ht->diag_sys_sw = *(pdata + offset);
    offset++;
    {
      std::string msg;
      uint8_t flg = 0x01;
      for(int i = 0;i<8;i++){
        uint8_t bit_i = flg<<i;
        bool sw = (bit_i & ht->diag_sys_sw)  == bit_i;
        msg += diag_sys_strs[i] + (sw?": On, ":":Off, ");
        if(i >= 5)
              break;
      }
      msg = msg.substr(0,msg.size() -2);
      info += "diag ctrl:\n" + msg + "\n";
    }

    memcpy(&ht->diag_hardware_sw, pdata + offset, 4);
    offset += 4;
    ht->diag_hardware_sw = ntohl(ht->diag_hardware_sw);
     {
      std::string msg;
      uint32_t flg = 0x01;
      for(int i = 0;i<32;i++){
        uint32_t bit_i = flg<<i;
        bool sw = (bit_i & ht->diag_hardware_sw)  == bit_i;
        msg += diag_hw_strs[i] + (sw?": On, ":":Off, ");
        if(i >= 15)
              break;
              if((i+1) %4 == 0)
                msg += "\n";
      }
      msg = msg.substr(0,msg.size() -2);
      info += "hardware diag switch:\n" +msg + "\n";
    }
  
    ht->dhcp_switch = *(pdata + offset);
    offset++;
    info += "dhcp switch: ";
    if(ht->dhcp_switch == 0x00 || ht->dhcp_switch == 0xFF){
      info += "Off";
    }else if(ht->dhcp_switch == 0x01){
      info += "On";
    }else{
      info += "Unknown";
    }
    info += "\n";

    memcpy(&ht->gateway[0], pdata + offset, 4);
    offset += 4;
    ResolveIp2String(&ht->gateway[0], tmp_str);
    info += "gateway: "+  tmp_str +"\n";

    ht->del_point_switch = *(pdata + offset);
    offset++;
    info += "del point switch: ";
    if(ht->del_point_switch == 0x00 || ht->del_point_switch == 0xFF){
      info += "Off";
    }else if(ht->del_point_switch == 0x01){
      info += "On";
    }else{
      info += "Unknown";
    }
    info += "\n";

    ht->adhesion_switch = *(pdata + offset);
    offset++;
    info += "adhesion switch: ";
    if(ht->adhesion_switch == 0x00 || ht->adhesion_switch == 0xFF){
      info += "Off";
    }else if(ht->adhesion_switch == 0x01){
      info += "On";
    }else{
      info += "Unknown";
    }
    info += "\n";

    ht->para_cfg_switch = *(pdata + offset);
    offset++;
    info += "para cfg switch: ";
    if(ht->para_cfg_switch == 0x00 || ht->para_cfg_switch == 0xFF){
      info += "Off";
    }else if(ht->para_cfg_switch == 0x01){
      info += "On";
    }else{
      info += "Unknown";
    }
    info += "\n";

    memcpy(&ht->unified_cfg_version[0], pdata + offset, 3);
    offset += 3;
   {
      char arr_c[128] = "";
      sprintf(arr_c, "%u.%u.%u", ht->unified_cfg_version[0], ht->unified_cfg_version[1], ht->unified_cfg_version[2]);
      tmp_str = std::string(arr_c);
   }
    info += "unified cfg version: "+  tmp_str +"\n";

    //reserved 3 bytes
    offset += 3;

    memcpy(&ht->mac_addr[0], pdata + offset, 6);
    offset += 6;
    ResolveMac2String(&ht->mac_addr[0], tmp_str);
    info += "mac addr: "+  tmp_str +"\n";
    
    ht->info = info;
   }

    return 0;
}

/** poll the heartbeat packet
 */
bool zvisionLidarMonitor::pollMsg(void)
{
  zvision_lidar_msgs::zvisionLidarHeartbeatPtr ht(new zvision_lidar_msgs::zvisionLidarHeartbeat);
  int ret = getPacket(ht);
  if (ret !=  0)
    return false; 

  // parsing udp packet
  if(parsingPacket(ht) != 0)
    return false;

  ht->stamp = ros::Time::now();
  if(config_.pub_msg && output_)
    output_.publish(ht);

  return true;
}

}