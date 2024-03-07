// MIT License
//
// Copyright(c) 2019 ZVISION. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

namespace zvision {

	enum EML30SPlusCmd {

		/* control */
		reboot,

		/* set */
		// network
		set_dhcp_switch,
		set_ip,
		set_gateway,
		set_netmask,
		set_mac,
		set_udp_dest_ip,
		set_udp_dest_port,
		// retro
		set_retro_switch,
		set_retro_target_gray_thre,
		set_retro_target_point_num_thre,
		set_retro_critical_point_dis_thre,
		set_retro_del_point_dis_low_thre,
		set_retro_del_point_dis_high_thre,
		set_retro_del_point_gray_thre,
		// adhesion
		set_adhesion_switch,
		set_adhesion_angle_hor_min,
		set_adhesion_angle_hor_max,
		set_adhesion_angle_ver_min,
		set_adhesion_angle_ver_max,
		set_adhesion_angle_hor_res,
		set_adhesion_angle_ver_res,
		set_adhesion_diff_thre,
		set_adhesion_dist_limit,
		set_adhesion_min_diff,
		// dirty detect
		set_dirty_detect_switch,
		set_dirty_detect_refresh,
		set_dirty_detect_cycle,
		set_dirty_detect_set_thre,
		set_dirty_detect_reset_thre,
		set_dirty_detect_inner_thre,
		set_dirty_detect_outer_thre,
		// delete point
		set_near_point_delete_switch,
		// other
		set_down_sample_mode,
		set_echo_mode,
		set_ptp_sync,
		set_frame_sync,
		set_frame_offset,
		set_angle_send,

		// cmd only send once above
		cmd_section_control,

		// send file
		set_ptp_file,
		set_json_config_file,

		// send data to device above
		cmd_section_send_data,


		/* read */
		read_serial_number,
		read_factory_mac,
		read_network_param,
		read_embeded_fpga_version,
		read_config_param_in_use,
		read_config_param_saved,
		read_algo_param_in_use,
		read_algo_param_saved,
		// read file
		read_ptp_file,
		read_json_config_file,
		read_cali_packets,
		read_temp_log,
		read_full_log,

		// receive data from device above
		cmd_section_receive_data
	};






};