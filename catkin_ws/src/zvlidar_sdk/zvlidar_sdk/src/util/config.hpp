#pragma once

#include <string>
#include <sstream>
#include <common/define.h>

namespace zv_processor{

    /* filter mode*/
    enum FilterMode
    {
        None = 0,
        Voxel = 1,
        Line = 2,
        OutlierRemoval = 3,
        ConfigFile = 4,
        Blooming = 5
    };

    struct FilterParam{
        FilterParam()
        :voxel_leaf_size(0.5f)
        ,line_sample(2)
        ,outlier_removal(0.25f)
        {}

        void setFilterMode(FilterMode mode)
        {
            filter_mode = mode;
            switch (mode)
            {
            case Voxel: filter_mode_str = "Voxel";break;
            case Line: filter_mode_str = "Line";break;
            case OutlierRemoval: filter_mode_str = "OutlierRemoval";break;
            case ConfigFile: filter_mode_str = "ConfigFile";break;
            case Blooming: filter_mode_str = "Blooming"; break;
            default: filter_mode_str = "None";break;
            }
        }

        void setFilterMode(std::string mode)
        {
            filter_mode_str = mode;
            updateFilterMode();
        }

        void updateFilterMode()
        {
            if(filter_mode_str.compare("Voxel") == 0)
                filter_mode = Voxel;
            else if(filter_mode_str.compare("Line") == 0)
                filter_mode = Line;
            else if(filter_mode_str.compare("OutlierRemoval") == 0)
                filter_mode = OutlierRemoval; 
            else if(filter_mode_str.compare("ConfigFile") == 0)
                filter_mode = ConfigFile;
            else if(filter_mode_str.compare("Blooming") == 0)
                filter_mode = Blooming;
            else
            {
                filter_mode = None;
                filter_mode_str = "None";
            }

        }

        FilterMode filter_mode;
        std::string filter_mode_str;
        /* filter parameters */
        std::string downsample_cfg_path;                // Read config file, customize downsampling
        uint16_t line_sample;                                        // Line
        float voxel_leaf_size;                                        // Voxel
        float outlier_removal;                                      // Outlier
    };

    /* yaml config */
    struct ProcessorConfig
    {
        /* basic */
        std::string lidar_type;
        std::string device_ip;
        uint16_t udp_port;
        std::string pcap_path;
        std::string angle_path;
        double pcap_rate;
        bool multicast_en;
        std::string multicast_ip;
        bool online;
        // debug
        float x_tra;
        float y_tra;
        float z_tra;
        float x_rot;
        float y_rot;
        float z_rot;

        /*ros*/
        std::string frame_id;
        std::string topic_tag_name;
        bool use_lidar_line_id;
        bool use_lidar_time;
        bool pub_lidar_packet;
        bool pub_pointcloud_xyzi;
        bool pub_pointcloud_xyzirt;
        bool pub_pointcloud_xyzrgba;
        bool pub_pointcloud_blooming;
        //diagnostic
        bool pub_diagnostic;

        /* proc */
        FilterParam filter_param;
    };

    /* diagnostic */
    #pragma pack(push)
    #pragma pack(1)
    struct LiarHeartbeat_ML30S_Factory
    {
        uint8_t tag[12];
        uint8_t version[2];
        uint8_t sn[18];
        uint8_t fpga_version[4];
        uint32_t fpga_file_size;
        uint8_t env_version[4];
        uint32_t env_file_size;
        uint8_t embadded_version[4];
        uint32_t embadded_file_size;
        uint8_t sys_diag_status;
        uint32_t hardware_diag_status;
        uint8_t ptp_sync_status;
        // reserved 2 bytes
        uint8_t res_1[2];
        uint8_t ip[4];
        uint32_t port;
        uint8_t time_sync_mode;
        uint8_t dst_ip[4];
        uint8_t retro_switch;
        uint8_t net_mask[4];
        uint8_t cfg_mac_addr[6];
        uint32_t frame_sync_offset;
        uint8_t echo_mode;
        uint8_t frame_sync_switch;
        uint8_t retro_intensity_percentage[2];
        uint8_t angle_send_switch;
        uint8_t downsample_mode;
        uint16_t dirty_check_thres_set_reset[2];
        uint8_t dirty_switch;
        uint8_t dirty_fresh_switch;
        uint16_t dirty_detect_cycle;
        uint8_t diag_switch;
        uint16_t diag_inner_thres;
        uint16_t diag_outer_thres;
        uint16_t point_loss_thres;
        uint8_t diag_sys_sw;
        uint32_t diag_hardware_sw;
        uint8_t dhcp_switch;
        uint8_t gateway[4];
        uint8_t del_point_switch;
        uint8_t adhesion_switch;
        uint8_t para_cfg_switch;
        uint8_t unified_cfg_version[3];
        // reserved 3 bytes
        uint8_t res_2[3];
        uint8_t mac_addr[6];
        // above 138 bytes, total 512bytes, left 374 bytes (512 - 138)
        uint8_t res_3[374];
    };

    struct LidarHeartbeat_ML30SPlus
    {
        /* data */
        uint8_t data[512];
    };

    struct LidarHeartbeat_ML30S
    {
        /* data */
        uint8_t data[512];
    };

    struct LidarHeartbeat_MLXS
    {
        /* data */
        uint8_t data[512];
    };
    #pragma pack(pop)

    struct DiagnosticInfo
    {
        DiagnosticInfo()
        :stamp_ns(0)
        ,dev_type(zvision::DeviceType::LidarUnknown)
        {}

        union SegmentInfo
        {
            /* ml30s  factory*/
            LiarHeartbeat_ML30S_Factory ml30s_factory;
            /* ml30s+ */
            LidarHeartbeat_ML30SPlus ml30s_plus;
            /* ml30s+ */
            LidarHeartbeat_ML30S ml30s;
            /* mlxs */
            LidarHeartbeat_MLXS mlxs;
        };

        /* description */
        uint64_t stamp_ns;                                        // nanosecond
        std::string ip_str;                                     // udp packet source ip address string
        std::string packet;                                   // heartbeat udp packet
        std::string info;                                        // parsed diagnostic information
        zvision::DeviceType dev_type;           // lidar type

        /* heartbeat packet */
        SegmentInfo seg_info;                         // diagnostic segment information for ML30S, ML30S+ and MLXS
    };
 
    inline std::string get_zv_logo()
    {
        std::stringstream ss;
        ss<<"/*\n";
        ss<<"***********************************************************\n";
        ss<<"* ######  #    #     #     ####      #     ####   #    #\n";
        ss<<"*     #   #    #     #    #          #    #    #  ##   #\n";
        ss<<"*    #    #    #     #     ####      #    #    #  # #  #\n";
        ss<<"*   #     #    #     #         #     #    #    #  #  # #\n";
        ss<<"*  #       #  #      #    #    #     #    #    #  #   ##\n";
        ss<<"* ######    ##       #     ####      #     ####   #    #\n";
        ss<<"***********************************************************\n";
        ss<<"*\n";
        ss<<"* Software License Agreement (Private License)\n";
        ss<<"*\n";
        ss<<"*  ZISION Tech - www.zvision.xyz.\n";
        ss<<"*  Copyright(c) 2018-2022 ZVISION.Co.Ltd\n";
        ss<<"*  All rights reserved.\n";
        ss<<"*/\n";
        return ss.str();
    }

    inline std::string get_switch_str(bool val)
    {
        return val?"True":"False";
    }

    inline uint8_t hex2uint8(char c) {
        uint8_t val = 0x0F;
        if (c >= 'A' && c <= 'Z')
            val = c - 'A' + 10;
        else if (c >= 'a' && c <= 'z')
            val = c - 'a' + 10;
        else if (c >= '0' && c <= '9')
            val = c - '0';
        return val;
    }
}