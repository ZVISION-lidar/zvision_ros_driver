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


#include "print.h"
#include "define.h"
#include "version.h"
namespace zvision
{
    // For ML30S+B1 EP(40ms) device
    static bool g_zvision_ml30splus_b1_ep_mode = false;

    std::string get_sdk_version_string() {
        return  std::string(ZVLIDAR_VERSION_STRING);;
    }

    std::string get_device_type_string(DeviceType tp)
    {
        std::string str = "Unknown";
        switch (tp)
        {
        case DeviceType::LidarML30B1:
            str = "ML30B1";
            break;
        case DeviceType::LidarML30SA1:
            str = "ML30SA1";
            break;
        case DeviceType::LidarML30SA1_2:
            str = "ML30SA1_2";
            break;
        case DeviceType::LidarML30SB1:
            str = "ML30SB1";
            break;
        case DeviceType::LidarML30SB2:
            str = "ML30SB2";
            break;
        case DeviceType::LidarMLX:
            str = "MLX";
            break;
        case DeviceType::LidarMLYA:
            str = "MLYA";
            break;
        case DeviceType::LidarMLYB:
            str = "MLYB";
            break;
        case DeviceType::LidarExciton:
            str = "EXCITON";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_device_type_string_by_mode(ScanMode sm)
    {
        std::string str = "Unknown";
        DeviceType tp = DeviceType::LidarUnknown;
        switch (sm)
        {
        case ScanMode::ScanML30B1_100:
            tp = DeviceType::LidarML30B1;
            break;
        case ScanMode::ScanML30SA1_160:
        case ScanMode::ScanML30SA1_160_1_2:
        case ScanMode::ScanML30SA1_160_1_4:
        case ScanMode::ScanML30SA1_190:
            tp = DeviceType::LidarML30SA1;
            break;
        case ScanMode::ScanMLX_160:
        case ScanMode::ScanMLX_190:
        case ScanMode::ScanMLXS_180:
            tp = DeviceType::LidarMLX;
            break;
        case ScanMode::ScanExciton:
            tp = DeviceType::LidarExciton;
            break;
        default:
            break;
        }
        return get_device_type_string(tp);
    }

    std::string get_scan_mode_string(ScanMode sm)
    {
        std::string str = "Unknown";
        switch (sm)
        {
        case ScanMode::ScanML30B1_100:
            str = "ML30 100";
            break;
        case ScanMode::ScanML30SA1_160:
            str = "ML30S 160";
            break;
        case ScanMode::ScanML30SA1_160_1_2:
            str = "ML30S 160(1/2)";
            break;
        case ScanMode::ScanML30SA1_160_1_4:
            str = "ML30S 160(1/4)";
            break;
        case ScanMode::ScanML30SA1_190:
            str = "ML30S 190";
            break;
        case ScanMode::ScanMLX_160:
            str = "MLX 160";
            break;
        case ScanMode::ScanMLX_190:
            str = "MLX 190";
            break;
        case ScanMode::ScanMLXS_180:
            str = "MLXS 180";
            break;
        case ScanMode::ScanExciton:
            str = "EXCITON";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_time_sync_type_string(TimestampType tp)
    {
        std::string str = "Unknown";
        switch (tp)
        {
        case TimestampType::TimestampPtp:
            str = "PTP";
            break;
        case TimestampType::TimestampPpsGps:
            str = "GPS";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_retro_mode_string(RetroMode tp)
    {
        std::string str = "Unknown";
        switch (tp)
        {
        case RetroMode::RetroDisable:
            str = "OFF";
            break;
        case RetroMode::RetroEnable:
            str = "ON";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_return_code_string(ReturnCode tp)
    {
        std::string str = "Unknown";
        switch (tp)
        {
        case ReturnCode::Success:
            str = "Success";
            break;
        case ReturnCode::Failure:
            str = "Failure";
            break;
        case ReturnCode::Timeout:
            str = "Timeout";
            break;
        case ReturnCode::InvalidParameter:
            str = "Invalid parameter";
            break;
        case ReturnCode::NotSupport:
            str = "Not support";
            break;
        case ReturnCode::InitSuccess:
            str = "Init success";
            break;
        case ReturnCode::InitFailure:
            str = "Init failure";
            break;
        case ReturnCode::NotInit:
            str = "Not init";
            break;
        case ReturnCode::OpenFileError:
            str = "Open file error";
            break;
        case ReturnCode::ReadFileError:
            str = "Read file error";
            break;
        case ReturnCode::InvalidContent:
            str = "Invalid content";
            break;
        case ReturnCode::EndOfFile:
            str = "End of file";
            break;
        case ReturnCode::NotMatched:
            str = "Not matched";
            break;
        case ReturnCode::BufferOverflow:
            str = "Buffer overflow";
            break;
        case ReturnCode::Unknown:
            str = "Unknown";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_echo_mode_string(EchoMode mode)
    {
        std::string str = "Unknown";
        switch (mode)
        {
        case EchoMode::EchoSingleFirst:
            str = "Single first return";
            break;
        case EchoMode::EchoSingleStrongest:
            str = "Singe strongest return";
            break;
        case EchoMode::EchoSingleLast:
            str = "Singe last return";
            break;
        case EchoMode::EchoDoubleFirstStrongest:
            str = "Double(first and strongest return)";
            break;
        case EchoMode::EchoDoubleFirstLast:
            str = "Double(first and last return)";
            break;
        case EchoMode::EchoDoubleStrongestLast:
            str = "Double(strongest and last return)";
            break;
        default:
            break;
        }
        return str;
    }

	std::string get_ml30splus_echo_mode_string(EchoMode mode)
	{
		std::string str = "Unknown";
		switch (mode)
		{
		case EchoMode::EchoSingleFirst:
		case EchoMode::EchoSingleStrongest:
		case EchoMode::EchoSingleLast:
			str = "Singe";
			break;
		case EchoMode::EchoDoubleFirstStrongest:
		case EchoMode::EchoDoubleFirstLast:
		case EchoMode::EchoDoubleStrongestLast:
			str = "Double";
			break;
		default:
			break;
		}
		return str;
	}

    std::string get_cfg_info_string(DeviceConfigurationInfo& info)
    {
        const int buffer_len = 4096;
        std::shared_ptr<char> buffer(new char[buffer_len]);
        char* ptr = buffer.get();
        int pos = 0;

		pos += snprintf(ptr + pos, buffer_len - pos, "Factory mac: %s\n", info.factory_mac.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Serial number: %s\n", info.serial_number.c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "DHCP mode: %s\n", zvision::get_state_mode_string(info.dhcp_enable).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Lidar ip: %s\n", info.device_ip.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Lidar subnet mask: %s\n", info.subnet_mask.c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Gateway address: %s\n", info.gateway_addr.c_str());
		// add Config mac
		pos += snprintf(ptr + pos, buffer_len - pos, "Current mac: %s\n", info.device_mac.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Config mac: %s\n", info.config_mac.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Destination ip: %s\n", info.destination_ip.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Destination port: %d\n", info.destination_port);
        pos += snprintf(ptr + pos, buffer_len - pos, "Timestamp sync mode: %s\n", zvision::get_time_sync_type_string(info.time_sync).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Retro enbale: %s\n", zvision::get_retro_mode_string(info.retro_enable).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Retro param: [%d] [%d]\n", info.retro_param_1_ref_min, info.retro_param_2_point_percent);
		pos += snprintf(ptr + pos, buffer_len - pos, "Adhesion enbale: %s\n", zvision::get_state_mode_string(info.adhesion_enable,true).c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Delete close point enbale: %s\n", zvision::get_state_mode_string(info.delete_point_enable, true).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Delete close point mode: %s\n", zvision::get_delete_point_mode_string(info.delete_point_mode).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Layer detection switch: %s\n", zvision::get_layer_sw_string_by_hard_diag_ctrl(info.hard_diag_ctrl).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Frame offset: %u(x 5ns)\n", info.phase_offset);
        pos += snprintf(ptr + pos, buffer_len - pos, "Frame offset sync: %s\n", zvision::get_phase_offset_mode_string(info.phase_offset_mode).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Dual return: %s\n", zvision::get_echo_mode_string(info.echo_mode).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Calibration File Broadcast: %s\n", zvision::get_cal_send_mode_string(info.cal_send_mode).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Downsample mode: %s\n", zvision::get_downsample_mode_string(info.downsample_mode).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "FPGA   version: %u.%u.%u.%u\n", info.version.boot_version[0], info.version.boot_version[1], info.version.boot_version[2], info.version.boot_version[3]);
        pos += snprintf(ptr + pos, buffer_len - pos, "Embedded version: %u.%u.%u.%u\n", info.version.kernel_version[0], info.version.kernel_version[1], info.version.kernel_version[2], info.version.kernel_version[3]);
        pos += snprintf(ptr + pos, buffer_len - pos, "FPGA   version(backup): %u.%u.%u.%u\n", info.backup_version.boot_version[0], info.backup_version.boot_version[1], info.backup_version.boot_version[2], info.backup_version.boot_version[3]);
        pos += snprintf(ptr + pos, buffer_len - pos, "Embedded version(backup): %u.%u.%u.%u\n", info.backup_version.kernel_version[0], info.backup_version.kernel_version[1], info.backup_version.kernel_version[2], info.backup_version.kernel_version[3]);
        //pos += snprintf(ptr + pos, buffer_len - pos, "Lidar type: %s\n", zvision::get_device_type_string(info.device).c_str());
        return std::string(ptr);
    }

	std::string get_ml30splus_cfg_info_string(DeviceConfigurationInfo& info)
	{
		const int buffer_len = 4096;
		std::shared_ptr<char> buffer(new char[buffer_len]);
		char* ptr = buffer.get();
		int pos = 0;

		pos += snprintf(ptr + pos, buffer_len - pos, "Factory mac: %s\n", info.factory_mac.c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Serial number: %s\n", info.serial_number.c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "DHCP mode: %s\n", zvision::get_state_mode_string(info.dhcp_enable).c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Lidar ip: %s\n", info.device_ip.c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Lidar subnet mask: %s\n", info.subnet_mask.c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Gateway address: %s\n", info.gateway_addr.c_str());
		// add Config mac
		pos += snprintf(ptr + pos, buffer_len - pos, "Current mac: %s\n", info.device_mac.c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Config mac: %s\n", info.config_mac.c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Destination ip: %s\n", info.destination_ip.c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Destination port: %d\n", info.destination_port);
		pos += snprintf(ptr + pos, buffer_len - pos, "Timestamp sync mode: %s\n", zvision::get_time_sync_type_string(info.time_sync).c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Retro enbale: %s\n", zvision::get_retro_mode_string(info.retro_enable).c_str());
		///pos += snprintf(ptr + pos, buffer_len - pos, "Retro param: [%d] [%d]\n", info.retro_param_1_ref_min, info.retro_param_2_point_percent);
		pos += snprintf(ptr + pos, buffer_len - pos, "Adhesion enbale: %s\n", zvision::get_state_mode_string(info.adhesion_enable, true).c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Delete close point enbale: %s\n", zvision::get_state_mode_string(info.delete_point_enable, true).c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Frame offset: %u(x 5ns)\n", info.phase_offset);
		pos += snprintf(ptr + pos, buffer_len - pos, "Frame offset sync: %s\n", zvision::get_phase_offset_mode_string(info.phase_offset_mode).c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Dual return: %s\n", zvision::get_echo_mode_string(info.echo_mode).c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Calibration File Broadcast: %s\n", zvision::get_cal_send_mode_string(info.cal_send_mode).c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "Downsample mode: %s\n", zvision::get_downsample_mode_string(info.downsample_mode).c_str());
		pos += snprintf(ptr + pos, buffer_len - pos, "FPGA   version: %u.%u.%u.%u\n", info.version.boot_version[0], info.version.boot_version[1], info.version.boot_version[2], info.version.boot_version[3]);
		pos += snprintf(ptr + pos, buffer_len - pos, "Embedded version: %u.%u.%u.%u\n", info.version.kernel_version[0], info.version.kernel_version[1], info.version.kernel_version[2], info.version.kernel_version[3]);
		pos += snprintf(ptr + pos, buffer_len - pos, "FPGA   version(backup): %u.%u.%u.%u\n", info.backup_version.boot_version[0], info.backup_version.boot_version[1], info.backup_version.boot_version[2], info.backup_version.boot_version[3]);
		pos += snprintf(ptr + pos, buffer_len - pos, "Embedded version(backup): %u.%u.%u.%u\n", info.backup_version.kernel_version[0], info.backup_version.kernel_version[1], info.backup_version.kernel_version[2], info.backup_version.kernel_version[3]);
		//pos += snprintf(ptr + pos, buffer_len - pos, "Lidar type: %s\n", zvision::get_device_type_string(info.device).c_str());
		return std::string(ptr);
	}

    std::string get_ml30splus_b1_cfg_info_string(DeviceConfigurationInfo& info)
    {
        const int buffer_len = 4096;
        std::shared_ptr<char> buffer(new char[buffer_len]);
        char* ptr = buffer.get();
        int pos = 0;

        pos += snprintf(ptr + pos, buffer_len - pos, "Factory mac: %s\n", info.factory_mac.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Serial number: %s\n", info.serial_number.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "DHCP mode: %s\n", zvision::get_state_mode_string(info.dhcp_enable).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Lidar ip: %s\n", info.device_ip.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Lidar subnet mask: %s\n", info.subnet_mask.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Gateway address: %s\n", info.gateway_addr.c_str());
        // add Config mac
        pos += snprintf(ptr + pos, buffer_len - pos, "Current mac: %s\n", info.device_mac.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Config mac: %s\n", info.config_mac.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Destination ip: %s\n", info.destination_ip.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Destination port: %d\n", info.destination_port);
        pos += snprintf(ptr + pos, buffer_len - pos, "PTP sync enable: %s\n", zvision::get_state_mode_string(info.ptp_sync_enable, true).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Retro enbale: %s\n", zvision::get_retro_mode_string(info.retro_enable).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Adhesion enbale: %s\n", zvision::get_state_mode_string(info.adhesion_enable, true).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "NoTarDel enbale: %s\n", zvision::get_state_mode_string(info.noTarDel_enable, true).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Delete close point enbale: %s\n", zvision::get_state_mode_string(info.delete_point_enable, true).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Dirty check enbale: %s\n", zvision::get_state_mode_string(info.dirty_check_enable, true).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Dirty refresh enbale: %s\n", zvision::get_state_mode_string(info.dirty_refresh_enable, true).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Intensity smooth enbale: %s\n", zvision::get_state_mode_string(info.intensity_smooth_enable, true).c_str());

        pos += snprintf(ptr + pos, buffer_len - pos, "Frame offset: %u(x 5ns)\n", info.phase_offset);
        pos += snprintf(ptr + pos, buffer_len - pos, "Frame offset sync: %s\n", zvision::get_phase_offset_mode_string(info.phase_offset_mode).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Dual return: %s\n", zvision::get_echo_mode_string(info.echo_mode).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Calibration File Broadcast: %s\n", zvision::get_cal_send_mode_string(info.cal_send_mode).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Downsample mode: %s\n", zvision::get_downsample_mode_string(info.downsample_mode).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "FPGA   version: %u.%u.%u.%u\n", info.version.boot_version[0], info.version.boot_version[1], info.version.boot_version[2], info.version.boot_version[3]);
        pos += snprintf(ptr + pos, buffer_len - pos, "Embedded version: %u.%u.%u.%u\n", info.version.kernel_version[0], info.version.kernel_version[1], info.version.kernel_version[2], info.version.kernel_version[3]);
        pos += snprintf(ptr + pos, buffer_len - pos, "FPGA   version(backup): %u.%u.%u.%u\n", info.backup_version.boot_version[0], info.backup_version.boot_version[1], info.backup_version.boot_version[2], info.backup_version.boot_version[3]);
        pos += snprintf(ptr + pos, buffer_len - pos, "Embedded version(backup): %u.%u.%u.%u\n", info.backup_version.kernel_version[0], info.backup_version.kernel_version[1], info.backup_version.kernel_version[2], info.backup_version.kernel_version[3]);
        //pos += snprintf(ptr + pos, buffer_len - pos, "Lidar type: %s\n", zvision::get_device_type_string(info.device).c_str());
        return std::string(ptr);
    }


    std::string get_phase_offset_mode_string(PhaseOffsetMode mode)
    {
        std::string str = "Unknown";
        switch (mode)
        {
        case PhaseOffsetMode::PhaseOffsetDisable:
            str = "OFF";
            break;
        case PhaseOffsetMode::PhaseOffsetEnable:
            str = "ON";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_cal_send_mode_string(CalSendMode mode)
    {
        std::string str = "Unknown";
        switch (mode)
        {
        case CalSendMode::CalSendDisable:
            str = "Disable";
            break;
        case CalSendMode::CalSendEnable:
            str = "Enable";
            break;
        default:
            break;
        }
        return str;
    }

	std::string get_state_mode_string(StateMode mode,bool onf)
	{
		std::string str = "Unknown";
		switch (mode)
		{
		case StateMode::StateDisable: {
			if(onf)
				str = "OFF";
			else
				str = "Disable";
		}break;
		case StateMode::StateEnable: {
			if (onf)
				str = "ON";
			else
				str = "Enable";
		}break;
		default:
			break;
		}
		return str;
	}

    std::string get_delete_point_mode_string(uint8_t mode)
    {
        std::string str = "Unknown";
        if (mode == 0x00)
            str = "force[0]";
        else if (mode == 0x01)
            str = "weak[1]";

        return str;
    }

    std::string get_layer_sw_string_by_hard_diag_ctrl(uint32_t flag)
    {
        std::string str;
        uint32_t layer_serious = 0x01 << 19;

        str += "serious";
        if ((flag & layer_serious) == layer_serious)
            str += "[1]";
        else
            str += "[0]";
        str += ", slight";
        uint32_t layer_slight = 0x01 << 20;
        if ((flag & layer_slight) == layer_slight)
            str += "[1]";
        else
            str += "[0]";

        return str;
    }

    std::string get_downsample_mode_string(DownsampleMode mode)
    {
        std::string str = "Unknown";
        switch (mode)
        {
        case DownsampleMode::DownsampleNone:
            str = "Downsample none";
            break;
        case DownsampleMode::Downsample_1_2:
            str = "Downsample 1/2";
            break;
        case DownsampleMode::Downsample_1_4:
            str = "Downsample 1/4";
            break;
        default:
            break;
        }
        return str;
    }


    bool is_lidar_heartbeat_packet(std::string pkt)
    {
        const int heart_beat_len = 48;
        std::string tag_30s = "ZVSHEARTBEAT";
        std::string tag_30sp = "zvision";
        std::string tag_30sp_b1 = "ZVSML30S+";
        std::string tag_mlxs_1 = "ZVSMLXS";
        std::string tag_mlxs_2 = "HEART";
        int len = pkt.size();
        if (len >= heart_beat_len)
        {
            // ml30s
            if (0 == tag_30s.compare(0, tag_30s.size(), pkt.substr(0, tag_30s.size())))
                return true;

            // ml30s+
            if(0 == tag_30sp.compare(0, tag_30sp.size(), pkt.substr(0, tag_30sp.size())))
                return true;

            // ml30s+ b1
            if (0 == tag_30sp_b1.compare(0, tag_30sp_b1.size(), pkt.substr(0, tag_30sp_b1.size())))
                return true;

            // mlxs
            if ((0 == tag_mlxs_1.compare(0, tag_mlxs_1.size(), pkt.substr(0, tag_mlxs_1.size()))) \
                && (0 == tag_mlxs_2.compare(0, tag_mlxs_2.size(), pkt.substr(15, tag_mlxs_2.size()))))
                return true;
        }
        return false;
    }

    uint16_t get_check_sum(std::string str)
    {
        uint32_t check_sum = 0;
        uint8_t* pstr = (uint8_t*)str.data();
        for (int i = 0; i < str.size(); i++)
            check_sum += *(pstr + i) & 0xFF;

        uint16_t ret = check_sum & 0xFFFF;
        return ret;
    }

    std::string get_lidar_frame_mark_string()
    {
        std::string str_x = std::string(50, 'x');
        std::string tag = std::string("zvision-lidar-mark-tag") + str_x;
        return tag;
    }

    void set_ml30splus_b1_ep_mode_enable(bool en) 
    {
        g_zvision_ml30splus_b1_ep_mode = en;
    }

    bool is_ml30splus_b1_ep_mode_enable()
    {
        return g_zvision_ml30splus_b1_ep_mode;
    }

} // end namespace zvision

