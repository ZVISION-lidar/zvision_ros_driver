### ML30S Lidar Heartbeat Packet
builtin_interfaces/Time                stamp      
uint8[512]    data
uint16            data_len 
string              info

string              tag
uint8[2]          version
string              sn

uint8[4]          fpga_version
uint32             fpga_file_size
uint8[4]          env_version
uint32             env_file_size
uint8[4]          embadded_version
uint32             embadded_file_size
uint8                 sys_diag_status
uint32             hardware_diag_status
uint8                ptp_sync_status
# reserved 2 bytes
uint8[4]           ip
uint32             port
uint8                time_sync_mode
uint8[4]           dst_ip
uint8                retro_switch
uint8[4]          net_mask
uint8[6]          cfg_mac_addr
uint32             frame_sync_offset
uint8                echo_mode
uint8                frame_sync_switch
uint8[2]              retro_intensity_percentage
uint8                 angle_send_switch
uint8                 downsample_mode
uint16[2]         dirty_check_thres_set_reset
uint8                 dirty_switch
uint8                 dirty_fresh_switch
uint16              dirty_detect_cycle
uint8                 diag_switch
uint16              diag_inner_thres
uint16              diag_outer_thres
uint16              point_loss_thres
uint8                 diag_sys_sw
uint32               diag_hardware_sw
uint8                 dhcp_switch
uint8[4]            gateway
uint8                 del_point_switch
uint8                 adhesion_switch
uint8                 para_cfg_switch
uint8[3]            unified_cfg_version
# reserved 3 bytes
uint8[6]             mac_addr