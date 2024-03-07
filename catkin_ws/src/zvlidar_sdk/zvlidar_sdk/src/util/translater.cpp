#include <util/translater.h>
#include <string.h>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <common/print.h>
#include <protocol/packet.h>



uint8_t g_color_table[256*4] ={
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

namespace zv_processor{

    /* read/write yaml file */
    template <typename T>
    void yamlReadValue(const YAML::Node& yaml, const std::string& key, T& out_val, const T& default_val)
    {
        if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
        {
            out_val = default_val;
        }
        else
        {
            out_val = yaml[key].as<T>();
        }
    }

    /* get configuration from yaml file */
    bool getProcessorsConfig(std::string path, std::vector<ProcessorConfig>& cfgs, std::string& infos)
    {
        /* load yaml file */
        YAML::Node context;
        try
        {
            context = YAML::LoadFile(path);
        }
        catch(...)
        {
            LOG_ERROR("Load yaml file failed.\n");
            return false;
        }

        std::string tag("lidar");
        YAML::Node node = context[tag.c_str()];
        if(!node)
            return false;

         /* get processors config */
        for(int i = 0; i<node.size(); i++)
        {
            // get basic parameters
            std::ostringstream info;
            ProcessorConfig cfg;
            tag = "base";
            YAML::Node base = node[i][tag.c_str()];
            if(!base)
                continue;
            
            info << "--- " << tag <<std::endl;
            yamlReadValue<std::string>(base, "lidar_type", cfg.lidar_type, "ML30S");
            info << "lidar_type:" << cfg.lidar_type <<std::endl;
            yamlReadValue<std::string>(base, "lidar_ip", cfg.device_ip, "192.168.10.108");
            info << "lidar_ip:" << cfg.device_ip <<std::endl;
            yamlReadValue<uint16_t>(base, "lidar_port", cfg.udp_port, 2368);
            info << "lidar_port:" << cfg.udp_port <<std::endl;
            yamlReadValue<std::string>(base, "pcap_path", cfg.pcap_path, "");
            info << "pcap_path:" << cfg.pcap_path <<std::endl;
            yamlReadValue<std::string>(base, "angle_path", cfg.angle_path, "");
            info << "angle_path:" << cfg.angle_path <<std::endl;
            yamlReadValue<double>(base, "pcap_rate", cfg.pcap_rate, 10);
            cfg.pcap_rate<0.1?0.1:cfg.pcap_rate;
            info << "pcap_rate:" << cfg.pcap_rate <<std::endl;
            yamlReadValue<bool>(base, "online", cfg.online, true);
            info << "online:" << get_switch_str(cfg.online) <<std::endl;
            yamlReadValue<bool>(base, "multicast_en", cfg.multicast_en, false);
            info << "multicast_en:" << get_switch_str(cfg.multicast_en) <<std::endl;
            yamlReadValue<std::string>(base, "multicast_ip", cfg.multicast_ip, "224.0.0.1");
            info << "multicast_ip:" << cfg.multicast_ip <<std::endl;
            yamlReadValue<float>(base, "x_tra", cfg.x_tra, 0);
            info << "x_tra:" << cfg.x_tra <<std::endl;
            yamlReadValue<float>(base, "y_tra", cfg.y_tra, 0);
            info << "y_tra:" << cfg.y_tra <<std::endl;
            yamlReadValue<float>(base, "z_tra", cfg.z_tra, 0);
            info << "z_tra:" << cfg.z_tra <<std::endl;
            yamlReadValue<float>(base, "x_rot", cfg.x_rot, 0);
            info << "x_rot:" << cfg.x_rot <<std::endl;
            yamlReadValue<float>(base, "y_rot", cfg.y_rot, 0);
            info << "y_rot:" << cfg.y_rot <<std::endl;
            yamlReadValue<float>(base, "z_rot", cfg.z_rot, 0);
            info << "z_rot:" << cfg.z_rot <<std::endl;
            
            tag = "ros";
            YAML::Node ros_node = node[i][tag.c_str()];
            if(!ros_node)
                continue;

            info << "--- " << tag << std::endl;
            yamlReadValue<std::string>(ros_node, "frame_id", cfg.frame_id, "/zvlidar");
            info << "frame_id:" << cfg.frame_id <<std::endl;
            yamlReadValue<std::string>(ros_node, "topic_tag_name", cfg.topic_tag_name, "zvlidar");
            info << "topic_tag_name:" << cfg.topic_tag_name <<std::endl;
            yamlReadValue<bool>(ros_node, "use_lidar_time", cfg.use_lidar_time, false);
            info << "use_lidar_time:" << get_switch_str(cfg.use_lidar_time) <<std::endl;
            yamlReadValue<bool>(ros_node, "use_lidar_line_id", cfg.use_lidar_line_id, false);
            info << "use_lidar_line_id:" << get_switch_str(cfg.use_lidar_line_id) <<std::endl;
            yamlReadValue<bool>(ros_node, "pub_pointcloud_blooming", cfg.pub_pointcloud_blooming, false);
            info << "pub_pointcloud_blooming:" << get_switch_str(cfg.pub_pointcloud_blooming) <<std::endl;
            yamlReadValue<bool>(ros_node, "pub_lidar_packet", cfg.pub_lidar_packet, false);
            info << "pub_lidar_packet:" << get_switch_str(cfg.pub_lidar_packet) <<std::endl;
            yamlReadValue<bool>(ros_node, "pub_pointcloud_xyzi", cfg.pub_pointcloud_xyzi, true);
            info << "pub_pointcloud_xyzi:" << get_switch_str(cfg.pub_pointcloud_xyzi) <<std::endl;
            yamlReadValue<bool>(ros_node, "pub_pointcloud_xyzirt", cfg.pub_pointcloud_xyzirt, false);
            info << "pub_pointcloud_xyzirt:" << get_switch_str(cfg.pub_pointcloud_xyzirt) <<std::endl;
            yamlReadValue<bool>(ros_node, "pub_pointcloud_xyzrgba", cfg.pub_pointcloud_xyzrgba, false);
            info << "pub_pointcloud_xyzrgba:" << get_switch_str(cfg.pub_pointcloud_xyzrgba) <<std::endl;
            yamlReadValue<bool>(ros_node, "pub_diagnostic", cfg.pub_diagnostic, false);
            info << "pub_diagnostic:" << cfg.pub_diagnostic <<std::endl;

            // add device config
            cfgs.push_back(cfg);
            ProcessorConfig& cfg_ = cfgs.back();
            // optional
            tag = "proc";
            YAML::Node proc = node[i][tag.c_str()];
            if(proc)
            {
                info << "--- " << tag <<std::endl;
                yamlReadValue<std::string>(proc, "filter_mode", cfg_.filter_param.filter_mode_str, "None");
                cfg_.filter_param.updateFilterMode();
                info << "filter_mode:" << cfg_.filter_param.filter_mode_str <<std::endl;
                yamlReadValue<float>(proc, "voxel_leaf_size", cfg_.filter_param.voxel_leaf_size, 0.5);
                info << "voxel_leaf_size:" << cfg_.filter_param.voxel_leaf_size <<std::endl;
                yamlReadValue<uint16_t>(proc, "line_sample", cfg_.filter_param.line_sample, 2);
                info << "line_sample:" << cfg_.filter_param.line_sample <<std::endl;
                yamlReadValue<float>(proc, "outlier_removal", cfg_.filter_param.outlier_removal, 0.25);
                info << "outlier_removal:" << cfg_.filter_param.outlier_removal <<std::endl;
                yamlReadValue<std::string>(proc, "downsample_cfg_path", cfg_.filter_param.downsample_cfg_path, "");
                info << "downsample_cfg_path:" << cfg_.filter_param.downsample_cfg_path <<std::endl;
            }

            infos += "------  lidar [ " + std::to_string(cfgs.size()) + " ] ------\n";
            infos += info.str();
            infos += "\n";
        }

        if(!cfgs.size())
            infos = "No device found in yaml config file.\n";
        else
            infos = "Found device count:[" + std::to_string(cfgs.size()) + "]\n" + infos;
    
        return cfgs.size() > 0;
    }

    void ResolveIp2String(const unsigned char* addr, std::string& ip)
    {
        char cip[128] = "";
        sprintf(cip, "%u.%u.%u.%u", addr[0], addr[1], addr[2], addr[3]);
        ip = std::string(cip);
    }
    /* get mac string by addr */
    void ResolveMac2String(const unsigned char* addr, std::string& ip)
    {
        char cip[128] = "";
        sprintf(cip, "%02X-%02X-%02X-%02X-%02X-%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        ip = std::string(cip);
    }
    
    /* parsing heartbeat packet */
    bool parsingDiagnosticInfo(DiagnosticInfo& diag)
    {
        
        int offset = 0;
        uint8_t* pdata = (uint8_t*)diag.packet.data();
         switch (diag.dev_type)
        {
            case zvision::DeviceType::LidarML30SA1:
            {
                /* todo  ...*/
                return false;
            }break;
            case zvision::DeviceType::LidarMl30SA1Plus:
            {
                /* todo  ...*/
                return false;
            }break;
            case zvision::DeviceType::LidarMl30SA1Factory:
            {
                // 
                std::string info;
                std::string tmp_str;
                // header
                auto ht = &diag.seg_info.ml30s_factory;
                memcpy(ht->tag, pdata,12);
                info += "tag: " +  std::string((char*)ht->tag) +"\n";
                offset = 12;

                memcpy(&ht->version[0], pdata + offset, 2);
                offset += 2;
                {
                    char cip[128] = "";
                    sprintf(cip, "%u.%u", ht->version[0], ht->version[1]);
                    info += "version: " +  std::string(cip) + "\n";
                }
                
                // mtd_info
                memcpy(ht->sn, pdata + offset,18);
                info += "sn: "+ std::string((char*)ht->sn) +"\n";
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
                
                diag.info = info;
            }break;
            case zvision::DeviceType::LidarMLX:
            {
                /* todo  ...*/
                return false;
            }break;
            
            default:
                return false;
        }

    }

#ifdef ENV_ROS
    /* convert mlxs lidar heartbeat packet to msg */
    // zvlidar_msgs_::MLXSHeartbeatMsg convertDiagToMLXSMsg(const DiagnosticInfo& diag)
    // {
    //     zvlidar_msgs_::MLXSHeartbeatMsg msg;
    //     if(sizeof(msg.data) != diag.packet.size())
    //     {
    //         return msg;
    //     }
// #ifdef ROS_FOUND
//             memcpy(msg.data.c_array(), diag.packet.data(), diag.packet.size());
// #else
//             memcpy(msg.data.data(), diag.packet.data(), diag.packet.size());
// #endif
    //     return msg;
    // }

    /* convert ml30s lidar heartbeat packet to msg */
    zvlidar_msgs_::ML30SHeartbeatMsg convertDiagToML30SMsg(const DiagnosticInfo& diag){
        zvlidar_msgs_::ML30SHeartbeatMsg msg;
        if(sizeof(msg.data) != diag.packet.size())
        {
            return msg;
        }
#ifdef ROS_FOUND
            memcpy(msg.data.c_array(), diag.packet.c_str(), diag.packet.size());
#else
            memcpy(msg.data.data(), diag.packet.c_str(), diag.packet.size());
#endif        
        return msg;
    }
    /* convert ml30s+ lidar heartbeat packet to msg */
    zvlidar_msgs_::ML30SPlusHeartbeatMsg convertDiagToML30SPlusMsg(const DiagnosticInfo& diag){

        zvlidar_msgs_::ML30SPlusHeartbeatMsg msg;
        if(sizeof(msg.data) != diag.packet.size())
        {
            return msg;
        }
#ifdef ROS_FOUND
            memcpy(msg.data.c_array(), diag.packet.c_str(), diag.packet.size());
#else
            memcpy(msg.data.data(), diag.packet.c_str(), diag.packet.size());
#endif            
        return msg;
    }
    /* convert ml30sp lidar heartbeat packet to msg */
    zvlidar_msgs_::ML30SFactoryHeartbeatMsg convertDiagToML30SFactoryMsg(const DiagnosticInfo& diag)
    {
        zvlidar_msgs_::ML30SFactoryHeartbeatMsg msg;
        
        time_t second = diag.stamp_ns/1e+9;
        msg.data_len = diag.packet.size();
        msg.info = diag.info;
        const LiarHeartbeat_ML30S_Factory& ht_info = diag.seg_info.ml30s_factory;
#ifdef ROS_FOUND
        //msg.header.stamp = ros::Time(second, (diag.stamp_ns - second*1e+9));
        msg.stamp = ros::Time(second, (diag.stamp_ns - second*1e+9));
        memcpy(msg.data.c_array(), diag.packet.data(), diag.packet.size());
        memcpy(msg.version.c_array(), ht_info.version, sizeof(ht_info.version));
        memcpy(msg.fpga_version.c_array(), ht_info.fpga_version, sizeof(ht_info.fpga_version));
        memcpy(msg.env_version.c_array(), ht_info.env_version, sizeof(ht_info.env_version));
        memcpy(msg.embadded_version.c_array(), ht_info.embadded_version, sizeof(ht_info.embadded_version));
        memcpy(msg.ip.c_array(), ht_info.ip, sizeof(ht_info.ip));
        memcpy(msg.dst_ip.c_array(), ht_info.dst_ip, sizeof(ht_info.dst_ip));
        memcpy(msg.net_mask.c_array(), ht_info.net_mask, sizeof(ht_info.net_mask));
        memcpy(msg.cfg_mac_addr.c_array(), ht_info.cfg_mac_addr, sizeof(ht_info.cfg_mac_addr));
        memcpy(msg.retro_intensity_percentage.c_array(), ht_info.retro_intensity_percentage, sizeof(ht_info.retro_intensity_percentage));
        memcpy(msg.dirty_check_thres_set_reset.c_array(), ht_info.dirty_check_thres_set_reset, sizeof(ht_info.dirty_check_thres_set_reset));
        memcpy(msg.gateway.c_array(), ht_info.gateway, sizeof(ht_info.gateway));
        memcpy(msg.unified_cfg_version.c_array(), ht_info.unified_cfg_version, sizeof(ht_info.unified_cfg_version));
        memcpy(msg.mac_addr.c_array(), ht_info.mac_addr, sizeof(ht_info.mac_addr));
#else
        msg.stamp = rclcpp::Time(second, (diag.stamp_ns - second*1e+9));
        memcpy(msg.data.data(), diag.packet.data(), diag.packet.size());
        memcpy(msg.version.data(), ht_info.version, sizeof(ht_info.version));
        memcpy(msg.fpga_version.data(), ht_info.fpga_version, sizeof(ht_info.fpga_version));
        memcpy(msg.env_version.data(), ht_info.env_version, sizeof(ht_info.env_version));
        memcpy(msg.embadded_version.data(), ht_info.embadded_version, sizeof(ht_info.embadded_version));
        memcpy(msg.ip.data(), ht_info.ip, sizeof(ht_info.ip));
        memcpy(msg.dst_ip.data(), ht_info.dst_ip, sizeof(ht_info.dst_ip));
        memcpy(msg.net_mask.data(), ht_info.net_mask, sizeof(ht_info.net_mask));
        memcpy(msg.cfg_mac_addr.data(), ht_info.cfg_mac_addr, sizeof(ht_info.cfg_mac_addr));
        memcpy(msg.retro_intensity_percentage.data(), ht_info.retro_intensity_percentage, sizeof(ht_info.retro_intensity_percentage));
        memcpy(msg.dirty_check_thres_set_reset.data(), ht_info.dirty_check_thres_set_reset, sizeof(ht_info.dirty_check_thres_set_reset));
        memcpy(msg.gateway.data(), ht_info.gateway, sizeof(ht_info.gateway));
        memcpy(msg.unified_cfg_version.data(), ht_info.unified_cfg_version, sizeof(ht_info.unified_cfg_version));
        memcpy(msg.mac_addr.data(), ht_info.mac_addr, sizeof(ht_info.mac_addr));
#endif
        
        msg.tag = std::string((char*)ht_info.tag);
        msg.sn = std::string((char*)ht_info.sn);
        msg.fpga_file_size = ht_info.fpga_file_size;
        msg.env_file_size = ht_info.env_file_size;
        msg.embadded_file_size = ht_info.embadded_file_size;
        msg.sys_diag_status = ht_info.sys_diag_status;
        msg.hardware_diag_status = ht_info.hardware_diag_status;
        msg.ptp_sync_status = ht_info.ptp_sync_status;
        msg.port = ht_info.port;
        msg.time_sync_mode = ht_info.time_sync_mode;
        msg.retro_switch = ht_info.retro_switch;
        msg.frame_sync_offset = ht_info.frame_sync_offset;
        msg.echo_mode = ht_info.echo_mode;
        msg.frame_sync_switch = ht_info.frame_sync_switch;
        msg.angle_send_switch = ht_info.angle_send_switch;
        msg.downsample_mode = ht_info.downsample_mode;
        msg.dirty_switch = ht_info.dirty_switch;
        msg.dirty_fresh_switch = ht_info.dirty_fresh_switch;
        msg.dirty_detect_cycle = ht_info.dirty_detect_cycle;
        msg.diag_switch = ht_info.diag_switch;
        msg.diag_inner_thres = ht_info.diag_inner_thres;
        msg.diag_outer_thres = ht_info.diag_outer_thres;
        msg.point_loss_thres = ht_info.point_loss_thres;
        msg.diag_sys_sw = ht_info.diag_sys_sw;
        msg.diag_hardware_sw = ht_info.diag_hardware_sw;
        msg.dhcp_switch = ht_info.dhcp_switch;
        msg.del_point_switch = ht_info.del_point_switch;
        msg.adhesion_switch = ht_info.adhesion_switch;
        msg.para_cfg_switch = ht_info.para_cfg_switch;
        return msg;
    }

    /* convert zvision points to packet message */
    void convertPoints2PacketMsg(const zvision::PointCloud& src, const MsgConvertParam& param, zvlidar_msgs_::ZvisionLidarScan& msg)
    {
        // get frame timestamp
        if(param.timestamp_type == TimeStampType::UseFirstUdpPacket)
        {
            for(auto pkt:src.packets){
                if(pkt.data.size()){
                    uint64_t stamp_ns = zvision::PointCloudPacket::GetTimestampNS(pkt.data);
                    time_t sec = stamp_ns/1000000000;
                    time_t nsec = stamp_ns%1000000000;
#ifdef ROS_FOUND
                    msg.header.stamp = ros::Time(sec, nsec);
#endif

#ifdef ROS2_FOUND
                    msg.header.stamp = rclcpp::Time(sec, nsec);
#endif
                    break;
                }
            }
        }
        else
        {
            time_t sec = src.stamp_ns/1000000000;
            time_t nsec = src.stamp_ns%1000000000;
#ifdef ROS_FOUND
                    msg.header.stamp = ros::Time(sec, nsec);
#endif

#ifdef ROS2_FOUND
                    msg.header.stamp = rclcpp::Time(sec, nsec);
#endif
        }
        // frame id
        msg.header.frame_id = param.frame_id;
        // packets
        
        msg.packets.resize(src.packets.size());

        int pos = 0;
        for(const auto& pkt:src.packets)
        {
#ifdef ROS_FOUND
            memcpy(msg.packets[pos].data.c_array(), pkt.data.data(), pkt.data.size());
#else
            memcpy(msg.packets[pos].data.data(), pkt.data.data(), pkt.data.size());
#endif
            
            pos++;
        }
    }
    /* convert zvision points to pointcloud message */
    void convertPoints2PointCloudMsg(const zvision::PointCloud& src, const MsgConvertParam& param, PointCloud2_& msg)
    {
        // get frame timestamp
        if(param.timestamp_type == TimeStampType::UseFirstUdpPacket)
        {
            for(auto pkt:src.packets){
                if(pkt.data.size()){
                    uint64_t stamp_ns = zvision::PointCloudPacket::GetTimestampNS(pkt.data);
                    time_t sec = stamp_ns/1000000000;
                    time_t nsec = stamp_ns%1000000000;
#ifdef ROS_FOUND
                    msg.header.stamp = ros::Time(sec, nsec);
#endif

#ifdef ROS2_FOUND
                    msg.header.stamp = rclcpp::Time(sec, nsec);
#endif
                    break;
                }
            }
        }
        else
        {
            time_t sec = src.stamp_ns/1000000000;
            time_t nsec = src.stamp_ns%1000000000;
#ifdef ROS_FOUND
                    msg.header.stamp = ros::Time(sec, nsec);
#endif

#ifdef ROS2_FOUND
                    msg.header.stamp = rclcpp::Time(sec, nsec);
#endif
        }

        // msg info
        int npoints = src.points.size();
        const uint8_t POINT_STEP = 32;
        const uint8_t BLOOMING_POINT_STEP = 60;
        msg.header.frame_id = param.frame_id;
        msg.height = 1;
        msg.width = npoints;
        if(param.pointcloud_type == PointCloudType::PointCloudBlooming)
        {
            msg.point_step = BLOOMING_POINT_STEP;
            msg.row_step = msg.width * 1 * BLOOMING_POINT_STEP;
        }
        else
        {
            msg.point_step = POINT_STEP;
            msg.row_step = msg.width * 1 * POINT_STEP;
        }
        msg.is_dense = false;
        msg.data.resize( msg.row_step * msg.height, 0x00);
        // set msg fields
        if(param.pointcloud_type== PointCloudType::PointCloudXYZRGBA)
        {
            msg.fields.resize(4);
            msg.fields[3].name = "rgba";
            msg.fields[3].offset = 16;
            msg.fields[3].datatype = PointField_::UINT32;
            msg.fields[3].count = 1;
        }
        else if(param.pointcloud_type== PointCloudType::PointCloudXYZIRT)
        {
            msg.fields.resize(6);
            msg.fields[3].name = "intensity";
            msg.fields[3].offset = 16;
            msg.fields[3].datatype = PointField_::UINT8;
            msg.fields[3].count = 1;
            msg.fields[4].name = "ring";
            msg.fields[4].offset = 18;
            msg.fields[4].datatype = PointField_::UINT16;
            msg.fields[4].count = 1;
            msg.fields[5].name = "timestamp";
            msg.fields[5].offset = 24;
            msg.fields[5].datatype = PointField_::FLOAT64;
            msg.fields[5].count = 1;
        }
        else if(param.pointcloud_type== PointCloudType::PointCloudBlooming)
        {
            msg.fields.resize(18);
            msg.fields[3].name = "intensity";
            msg.fields[3].offset = 16;
            msg.fields[3].datatype = PointField_::UINT8;
            msg.fields[3].count = 1;
            msg.fields[4].name = "xd";
            msg.fields[4].offset = 17;
            msg.fields[4].datatype = PointField_::FLOAT32;
            msg.fields[4].count = 1;
            msg.fields[5].name = "yd";
            msg.fields[5].offset = 21;
            msg.fields[5].datatype = PointField_::FLOAT32;
            msg.fields[5].count = 1;
            msg.fields[6].name = "zd";
            msg.fields[6].offset = 25;
            msg.fields[6].datatype = PointField_::FLOAT32;
            msg.fields[6].count = 1;
            msg.fields[7].name = "id";
            msg.fields[7].offset = 29;
            msg.fields[7].datatype = PointField_::UINT16;
            msg.fields[7].count = 1;
            msg.fields[8].name = "peak";
            msg.fields[8].offset = 31;
            msg.fields[8].datatype = PointField_::UINT8;
            msg.fields[8].count = 1;
            msg.fields[9].name = "pulse_width";
            msg.fields[9].offset = 32;
            msg.fields[9].datatype = PointField_::FLOAT32;
            msg.fields[9].count = 1;
            msg.fields[10].name = "gain";
            msg.fields[10].offset = 36;
            msg.fields[10].datatype = PointField_::UINT8;
            msg.fields[10].count = 1;
            msg.fields[11].name = "noisy";
            msg.fields[11].offset = 37;
            msg.fields[11].datatype = PointField_::FLOAT32;
            msg.fields[11].count = 1;
            msg.fields[12].name = "dc";
            msg.fields[12].offset = 41;
            msg.fields[12].datatype = PointField_::FLOAT32;
            msg.fields[12].count = 1;
            msg.fields[13].name = "ftof_max";
            msg.fields[13].offset = 45;
            msg.fields[13].datatype = PointField_::UINT8;
            msg.fields[13].count = 1;
            msg.fields[14].name = "ftof";
            msg.fields[14].offset = 46;
            msg.fields[14].datatype = PointField_::FLOAT32;
            msg.fields[14].count = 1;
            msg.fields[15].name = "azimuth";
            msg.fields[15].offset = 50;
            msg.fields[15].datatype = PointField_::FLOAT32;
            msg.fields[15].count = 1;
            msg.fields[16].name = "elevation";
            msg.fields[16].offset = 54;
            msg.fields[16].datatype = PointField_::FLOAT32;
            msg.fields[16].count = 1;
            msg.fields[17].name = "channel";
            msg.fields[17].offset = 58;
            msg.fields[17].datatype = PointField_::UINT16;
            msg.fields[17].count = 1;

        }
        else
        /* default: PointCloudXYZI*/
        {
            msg.fields.resize(4);
            msg.fields[3].name = "intensity";
            msg.fields[3].offset = 16;
            msg.fields[3].datatype = PointField_::FLOAT32;
            msg.fields[3].count = 1;
        }
        
        // field x,y,z
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = PointField_::FLOAT32;
        msg.fields[0].count = 1;
        msg.fields[1].name = "y";
        msg.fields[1].offset = 4;
        msg.fields[1].datatype = PointField_::FLOAT32;
        msg.fields[1].count = 1;
        msg.fields[2].name = "z";
        msg.fields[2].offset = 8;
        msg.fields[2].datatype = PointField_::FLOAT32;
        msg.fields[2].count = 1;

        uint8_t *p_data = msg.data.data();
        for (size_t i = 0; i < npoints; i++)
        {
            *(reinterpret_cast<float*>(p_data +  0)) = src.points[i].x;
            *(reinterpret_cast<float*>(p_data +  4)) = src.points[i].y;
            *(reinterpret_cast<float*>(p_data +  8)) = src.points[i].z;

            if(param.pointcloud_type== PointCloudType::PointCloudXYZRGBA)
            {
                // now we get point color from colormap
                uint8_t intensity = src.points[i].reflectivity & 0xFF;
                *(reinterpret_cast<uint8_t*>(p_data +  18)) = g_color_table[intensity *3 + 0]; // R
                *(reinterpret_cast<uint8_t*>(p_data +  17)) = g_color_table[intensity *3 + 1]; // G
                *(reinterpret_cast<uint8_t*>(p_data +  16)) = g_color_table[intensity *3 + 2]; // B
                *(reinterpret_cast<uint8_t*>(p_data +  19)) = 0xFF; // A
            }
            else if(param.pointcloud_type== PointCloudType::PointCloudXYZIRT)
            {
                *(reinterpret_cast<uint8_t*>(p_data +  16)) = src.points[i].reflectivity & 0xFF;
                if(param.ring_type == RingType::FovId)
                    *(reinterpret_cast<uint16_t*>(p_data +  18)) = src.points[i].fov;
                else if(param.ring_type == RingType::LineId)
                    *(reinterpret_cast<uint16_t*>(p_data +  18)) = src.points[i].line_id;
                else
                    *(reinterpret_cast<uint16_t*>(p_data +  18)) = 0;

                *(reinterpret_cast<double*>(p_data +  24)) = 1e-9 * src.points[i].timestamp_ns;
            }
            else if(param.pointcloud_type== PointCloudType::PointCloudBlooming)
            {
                // set blooming data
                *(reinterpret_cast<uint8_t*>(p_data +  16)) = src.points[i].reflectivity & 0xFF;
                if(src.use_blooming)
                {
                    *(reinterpret_cast<float*>(p_data +  17)) = src.blooming_frame->points[i].x;
                    *(reinterpret_cast<float*>(p_data +  21)) = src.blooming_frame->points[i].y;
                    *(reinterpret_cast<float*>(p_data +  25)) = src.blooming_frame->points[i].z;
                    *(reinterpret_cast<uint16_t*>(p_data +  29)) = src.blooming_frame->points[i].reflectivity_13bits;
                    *(reinterpret_cast<uint8_t*>(p_data +  31)) = src.blooming_frame->points[i].peak;
                    *(reinterpret_cast<float*>(p_data +  32)) = src.blooming_frame->points[i].pulse_width;
                    *(reinterpret_cast<uint8_t*>(p_data +  36)) = src.blooming_frame->points[i].gain;
                    *(reinterpret_cast<float*>(p_data +  37)) = src.blooming_frame->points[i].noisy;
                    *(reinterpret_cast<float*>(p_data +  41)) = src.blooming_frame->points[i].direct_current;
                    *(reinterpret_cast<uint8_t*>(p_data +  45)) = src.blooming_frame->points[i].ftof_max;
                    *(reinterpret_cast<float*>(p_data +  46)) = src.blooming_frame->points[i].ftof;
                    *(reinterpret_cast<float*>(p_data +  50)) = src.points[i].azi;
                    *(reinterpret_cast<float*>(p_data +  54)) = src.points[i].ele;
                    *(reinterpret_cast<uint16_t*>(p_data +  58)) = src.points[i].channel;
                }
            }
            else
            {
                *(reinterpret_cast<float*>(p_data +  16)) = src.points[i].reflectivity & 0xFF;
            }

            if(param.pointcloud_type== PointCloudType::PointCloudBlooming)
                p_data += BLOOMING_POINT_STEP;
            else
                p_data += POINT_STEP;
        }
    }
#endif
}

