#include <util/translater.h>
#include <string.h>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <common/print.h>
#include <protocol/packet.h>

#define DEG2RAD(x)  (x*3.1415926/180)

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
        for(int i = 0; i< int(node.size()); i++)
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
            yamlReadValue<std::string>(base, "transport", cfg.transport, "udp");
            info << "transport:" << cfg.transport <<std::endl;
            yamlReadValue<std::string>(base, "lidar_ip", cfg.device_ip, "192.168.10.108");
            info << "lidar_ip:" << cfg.device_ip <<std::endl;
            yamlReadValue<uint16_t>(base, "lidar_port", cfg.udp_port, 2368);
            info << "lidar_port:" << cfg.udp_port <<std::endl;
            yamlReadValue<std::string>(base, "pcap_path", cfg.pcap_path, "");
            info << "pcap_path:" << cfg.pcap_path <<std::endl;
            yamlReadValue<std::string>(base, "angle_path", cfg.angle_path, "");
            info << "angle_path:" << cfg.angle_path <<std::endl;
            yamlReadValue<double>(base, "pcap_rate", cfg.pcap_rate, 10);
            cfg.pcap_rate = cfg.pcap_rate<0.1?0.1:cfg.pcap_rate;
            info << "pcap_rate:" << cfg.pcap_rate <<std::endl;
            yamlReadValue<bool>(base, "online", cfg.online, true);
            info << "online:" << get_switch_str(cfg.online) <<std::endl;
            yamlReadValue<bool>(base, "multicast_en", cfg.multicast_en, false);
            info << "multicast_en:" << get_switch_str(cfg.multicast_en) <<std::endl;
            yamlReadValue<std::string>(base, "multicast_ip", cfg.multicast_ip, "224.0.0.1");
            info << "multicast_ip:" << cfg.multicast_ip <<std::endl;
            /* MRZ16 serial fields (used when transport == "serial") */
            yamlReadValue<std::string>(base, "port_send", cfg.port_send, "/dev/ttyACM2");
            info << "port_send:" << cfg.port_send <<std::endl;
            yamlReadValue<std::string>(base, "port_recv", cfg.port_recv, "/dev/ttyACM0");
            info << "port_recv:" << cfg.port_recv <<std::endl;
            yamlReadValue<int>(base, "baud_send", cfg.baud_send, 9600);
            info << "baud_send:" << cfg.baud_send <<std::endl;
            yamlReadValue<int>(base, "baud_recv", cfg.baud_recv, 3125000);
            info << "baud_recv:" << cfg.baud_recv <<std::endl;
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
            yamlReadValue<bool>(ros_node, "pub_lidar_packet", cfg.pub_lidar_packet, false);
            info << "pub_lidar_packet:" << get_switch_str(cfg.pub_lidar_packet) <<std::endl;
            yamlReadValue<bool>(ros_node, "pub_pointcloud_xyzi", cfg.pub_pointcloud_xyzi, true);
            info << "pub_pointcloud_xyzi:" << get_switch_str(cfg.pub_pointcloud_xyzi) <<std::endl;
            yamlReadValue<bool>(ros_node, "pub_pointcloud_xyzirt", cfg.pub_pointcloud_xyzirt, false);
            info << "pub_pointcloud_xyzirt:" << get_switch_str(cfg.pub_pointcloud_xyzirt) <<std::endl;
            yamlReadValue<bool>(ros_node, "pub_pointcloud_xyzrgba", cfg.pub_pointcloud_xyzrgba, false);
            info << "pub_pointcloud_xyzrgba:" << get_switch_str(cfg.pub_pointcloud_xyzrgba) <<std::endl;
            yamlReadValue<bool>(ros_node, "pub_imu_msg", cfg.pub_imu_msg, false);
            info << "pub_imu_msg:" << get_switch_str(cfg.pub_imu_msg) <<std::endl;

            // add device config
            cfgs.push_back(cfg);

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

#ifdef ENV_ROS
    /* convert zvision points to packet message */
    void convertPoints2PacketMsg(const zvision::PointCloud& src, const MsgConvertParam& param, zvlidar_msgs_::ZvisionLidarScan& msg)
    {

#if 0
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
        #ifdef ROS_FOUND
            msg.header.stamp = ros::Time::now();
        #endif

        #ifdef ROS2_FOUND
            msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
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
#endif

    }
    /* convert zvision points to pointcloud message */
    void convertPoints2PointCloudMsg(const zvision::PointCloud& src, const MsgConvertParam& param, PointCloud2_& msg, ProcessorConfig& cfg)
    {
        // get frame timestamp
        if(param.timestamp_type == TimeStampType::UseFirstUdpPacket)
        {
            uint64_t stamp_us = src.points[0].timestamp_us;
            time_t sec = stamp_us/1000000;
            time_t nsec = (stamp_us%1000000)*1000;

#ifdef ROS_FOUND
            msg.header.stamp = ros::Time(sec, nsec);
#endif

#ifdef ROS2_FOUND
            msg.header.stamp = rclcpp::Time(sec, nsec);
#endif
            // LOG_INFO("time = %d %d\n",sec,nsec);
        }
        else
        {
#ifdef ROS_FOUND
            ros::Time now = ros::Time::now();
            msg.header.stamp = now;
#endif

#ifdef ROS2_FOUND
            rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
            msg.header.stamp = now;
#endif
        }

        // msg info
        int npoints = src.points.size();
        const uint8_t POINT_STEP = 32;
        msg.header.frame_id = param.frame_id;
        msg.height = 1;
        msg.width = npoints;
        msg.point_step = POINT_STEP;
        msg.row_step = msg.width * 1 * POINT_STEP;
        msg.is_dense = true;
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
        for (int i = 0; i < npoints; i++)
        {
            double Ax = cfg.x_rot / 180.0 * M_PI;
            double Ay = cfg.y_rot / 180.0 * M_PI;
            double Az = cfg.z_rot / 180.0 * M_PI;
            double x,y,z;

            zvision::Point sourPoint = src.points[i];
            zvision::Point xTransPoint = src.points[i];
            zvision::Point yTransPoint = src.points[i];
            zvision::Point zTransPoint = src.points[i];

            // X rotation 
            x = sourPoint.x;
            y = sourPoint.y;
            z = sourPoint.z;
            xTransPoint.x = x;
            xTransPoint.y = y * std::cos(Ax) - z * std::sin(Ax);
            xTransPoint.z = y * std::sin(Ax) + z * std::cos(Ax);

            // Y roration
            x = xTransPoint.x;
            y = xTransPoint.y;
            z = xTransPoint.z;
            yTransPoint.x = x * std::cos(Ay) + z * std::sin(Ay);
            yTransPoint.y = y;
            yTransPoint.z = x * (-std::sin(Ay)) + z * std::cos(Ay);

            // Z roration
            x = yTransPoint.x;
            y = yTransPoint.y;
            z = yTransPoint.z;
            zTransPoint.x = x * std::cos(Az) + y * (-std::sin(Az));
            zTransPoint.y = x * std::sin(Az) + y * std::cos(Az);
            zTransPoint.z = z;

            *(reinterpret_cast<float*>(p_data +  0)) = zTransPoint.x + cfg.x_tra;
            *(reinterpret_cast<float*>(p_data +  4)) = zTransPoint.y + cfg.y_tra;
            *(reinterpret_cast<float*>(p_data +  8)) = zTransPoint.z + cfg.z_tra;

            // *(reinterpret_cast<float*>(p_data +  0)) = src.points[i].x;
            // *(reinterpret_cast<float*>(p_data +  4)) = src.points[i].y;
            // *(reinterpret_cast<float*>(p_data +  8)) = src.points[i].z;

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
                *(reinterpret_cast<uint8_t*>(p_data +  18)) = src.points[i].row;
                uint64_t ts_us;
                if(param.timestamp_type == TimeStampType::UseFirstUdpPacket)
                {
                    ts_us = src.points[i].timestamp_us;
                }
                else
                {
#ifdef ROS_FOUND
                    ros::Time now = ros::Time::now();
                    ts_us = static_cast<uint64_t>(now.sec) * 1000000 + now.nsec / 1000;
#endif
#ifdef ROS2_FOUND
                    rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
                    ts_us = now.nanoseconds() / 1000;
#endif
                }
                *(reinterpret_cast<double*>(p_data +  24)) = 1e-6 * ts_us;
            }
            else
            {
                *(reinterpret_cast<float*>(p_data +  16)) = src.points[i].reflectivity & 0xFF;
            }

            p_data += POINT_STEP;
        }
    }

    void convertImudata2ImuMsg(const zvision::imu_data_t & src,Imu_msg_& msg,const MsgConvertParam& param,ProcessorConfig& cfg)
    {
        if(param.timestamp_type == TimeStampType::UseFirstUdpPacket)
        {
            uint64_t stamp_us = src.timestamp_us;
            time_t sec = stamp_us/1000000;
            time_t nsec = (stamp_us%1000000)*1000;

#ifdef ROS_FOUND
            msg.header.stamp = ros::Time(sec, nsec);
#endif

#ifdef ROS2_FOUND
            msg.header.stamp = rclcpp::Time(sec, nsec);
#endif
            // LOG_INFO("time = %d %d\n",sec,nsec);
        }
        else
        {
#ifdef ROS_FOUND
            msg.header.stamp = ros::Time::now();
#endif

#ifdef ROS2_FOUND
            msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
#endif
        }

        if (cfg.lidar_type == "nz1_a2")
        {
            // NZ1_A2: 坐标系需取反并交换轴 (x:-acc_x, y:-acc_z, z:-acc_y)
            msg.linear_acceleration.x = -src.acc_x;
            msg.linear_acceleration.y = -src.acc_z;
            msg.linear_acceleration.z = -src.acc_y;

            msg.angular_velocity.x = -DEG2RAD(src.gyro_x);
            msg.angular_velocity.y = -DEG2RAD(src.gyro_z);
            msg.angular_velocity.z = -DEG2RAD(src.gyro_y);
        }
        else
        {
            // 其他机型: 坐标系一致, 直接 1:1 赋值
            msg.linear_acceleration.x = src.acc_x;
            msg.linear_acceleration.y = src.acc_y;
            msg.linear_acceleration.z = src.acc_z;

            msg.angular_velocity.x = DEG2RAD(src.gyro_x);
            msg.angular_velocity.y = DEG2RAD(src.gyro_y);
            msg.angular_velocity.z = DEG2RAD(src.gyro_z);
        }
    }

#endif
}

