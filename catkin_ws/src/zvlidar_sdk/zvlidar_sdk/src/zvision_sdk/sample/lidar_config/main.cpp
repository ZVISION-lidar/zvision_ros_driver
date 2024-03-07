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
#include "lidar_tools.h"
#include "define.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <set>
#include <thread>
#include <memory>
#include "convert.hpp"
#include "loguru.hpp"
/* Progress bar defination */
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 50
/* Progress information */
static std::map<std::string, int> g_lidars_percent;

//Callback function for progress notify
void print_current_progress(std::string ip, int percent)
{
    LOG_F(INFO, "#%s:Current progress %3d.", ip.c_str(), percent);
}

/* Callback function for progress notifiy with progress bar */
void print_current_progress_multi(int percent, std::string ip)
{
	g_lidars_percent[ip] = percent;
	//get min percent
	int min_percent = percent;
	for (auto it : g_lidars_percent)
		if (it.second < min_percent) min_percent = it.second;

	int lpad = (int)(1.0f * min_percent / 100 * PBWIDTH);
	int rpad = PBWIDTH - lpad;
	int idx = 0;
	for (auto it : g_lidars_percent) {

		if (idx == 0)
			printf("\r\r #%s:[%3d%%] ", it.first.c_str(), it.second);
		else
			printf("#%s:[%3d%%] ", it.first.c_str(), it.second);

		if (idx == (g_lidars_percent.size() - 1))
			printf(" [%.*s%*s]", lpad, PBSTR, rpad, "");
		idx++;
	}
	fflush(stdout);
}

//Sample code 0 : Set lidar's mac address
int sample_config_lidar_mac_address(std::string lidar_ip, std::string mac, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.mac = mac;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_mac, &param_30sp);
	}
	else {
		ret = config.SetDeviceMacAddress(mac);
	}
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s MAC address to [%s] failed, ret = %d.", lidar_ip.c_str(), mac.c_str(), ret);
    else
        LOG_F(INFO, "Set device [%s]'s MAC address to [%s] ok.", lidar_ip.c_str(), mac.c_str());
    return ret;
}

//Sample code 1 : Set lidar's static ip address
int sample_config_lidar_ip(std::string lidar_ip, std::string new_ip, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.ip = new_ip;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_ip, &param_30sp);
	}
	else {
		ret = config.SetDeviceStaticIpAddress(new_ip);
	}
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s IP address to [%s] failed, ret = %d.", lidar_ip.c_str(), new_ip.c_str(), ret);
    else
        LOG_F(INFO, "Set device [%s]'s IP address to [%s] ok.", lidar_ip.c_str(), new_ip.c_str());
    return ret;
}

//Sample code 2 : Set lidar's subnet mask
int sample_config_lidar_subnet_mask(std::string lidar_ip, std::string subnetmask, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.netmask = subnetmask;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_netmask, &param_30sp);
	}
	else {
		ret = config.SetDeviceSubnetMask(subnetmask);
	}

    if (ret)
        LOG_F(ERROR, "Set device [%s]'s subnet mask to [%s] failed, ret = %d.", lidar_ip.c_str(), subnetmask.c_str(), ret);
    else
        LOG_F(INFO, "Set device [%s]'s subnet mask to [%s] ok.", lidar_ip.c_str(), subnetmask.c_str());
    return ret;
}

//Sample code 3 : Set lidar's udp destination ip address
int sample_config_lidar_udp_destination_ip(std::string lidar_ip, std::string dst_ip, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.udp_dest_ip = dst_ip;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_udp_dest_ip, &param_30sp);
	}
	else {
		ret = config.SetDeviceUdpDestinationIpAddress(dst_ip);
	}
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s UDP destination ip to [%s] failed, ret = %d.", lidar_ip.c_str(), dst_ip.c_str(), ret);
    else
        LOG_F(INFO, "Set device [%s]'s UDP destination ip to [%s] ok.", lidar_ip.c_str(), dst_ip.c_str());
    return ret;
}

//Sample code 4 : Set lidar's udp destination port
int sample_config_lidar_udp_destination_port(std::string lidar_ip, int dst_port, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.udp_dest_port = dst_port;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_udp_dest_port, &param_30sp);
	}
	else {
		ret = config.SetDeviceUdpDestinationPort(dst_port);
	}
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s UDP destination port to [%d] failed, ret = %d.", lidar_ip.c_str(), dst_port, ret);
    else
        LOG_F(INFO, "Set device [%s]'s UDP destination port to [%d] ok.", lidar_ip.c_str(), dst_port);
    return ret;
}

//Sample code 5 : Set lidar's retro function
int sample_config_lidar_retro_enable(std::string lidar_ip, bool enable, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.switch_retro = enable;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_retro_switch, &param_30sp);
	}
	else {
		ret = config.SetDeviceRetroEnable(enable);
	}
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s retro to [%d] failed, ret = %d.", lidar_ip.c_str(), enable, ret);
    else
        LOG_F(INFO, "Set device [%s]'s retro to [%d] ok.", lidar_ip.c_str(), enable);
    return ret;
}

//Sample code 6 : Set lidar's time sync mode
int sample_config_lidar_time_sync(std::string lidar_ip, zvision::TimestampType type)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceTimestampType(type))
        LOG_F(ERROR, "Set device [%s]'s timestamp type to [%d] failed, ret = %d.", lidar_ip.c_str(), type, ret);
    else
        LOG_F(INFO, "Set device [%s]'s timestamp type to [%d] ok.", lidar_ip.c_str(), type);
    return ret;
}

//Sample code 7 : Query lidar's firmware version
int sample_query_lidar_firmware_version(std::string lidar_ip, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    std::string boot_version, kernel_version;
    zvision::FirmwareVersion version;
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::read_embeded_fpga_version, &param_30sp);
		memcpy(version.kernel_version, param_30sp.embedded_ver, 4);
		memcpy(version.boot_version, param_30sp.fpga_ver, 4);
	}
	else {
		ret = config.QueryDeviceFirmwareVersion(version);
	}
    if (ret)
        LOG_F(ERROR, "Query device [%s]'s firmware version failed, ret = %d.", lidar_ip.c_str(), ret);
    else
    {
        LOG_F(INFO, "Query device [%s]'s firmware version ok.", lidar_ip.c_str());
        LOG_F(INFO, "FPGA   version: %u.%u.%u", version.boot_version[0], version.boot_version[1], version.boot_version[2], version.boot_version[3]);
        LOG_F(INFO, "Embedded version: %u.%u.%u", version.kernel_version[0], version.kernel_version[1], version.kernel_version[2], version.kernel_version[3]);
    }
    return ret;
}

//Sample code 8 : Query lidar's serial number
int sample_query_lidar_serial_number(std::string lidar_ip, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    std::string serial_number;
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::read_serial_number, &param_30sp);
		serial_number = param_30sp.serial_number;
	}
	else {
		ret = config.QueryDeviceSnCode(serial_number);
	}
    if (ret)
        LOG_F(ERROR, "Query device [%s]'s serial number failed, ret = %d.", lidar_ip.c_str(), ret);
    else
    {
        LOG_F(INFO, "Query device [%s]'s serial number ok.", lidar_ip.c_str());
        LOG_F(INFO, "Serial number: %s", serial_number.c_str());
    }
    return ret;
}

//Sample code 9 : Query lidar's hardware temperature
int sample_query_lidar_hardware_temperature(std::string lidar_ip)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    float temperature0 = 0.0, temperature1 = 0.0;
    if (ret = config.QueryDeviceTemperature(temperature0, temperature1))
        LOG_F(ERROR, "Query device [%s]'s hardware temperature failed, ret = %d.", lidar_ip.c_str(), ret);
    else
    {
        LOG_F(INFO, "Query device [%s]'s hardware temperature ok.", lidar_ip.c_str());
        LOG_F(INFO, "Temperature 0: %.3f", temperature0);
        LOG_F(INFO, "Temperature 1: %.3f", temperature1);
    }
    return ret;
}

//Sample code 10 : Query lidar's configurature
int sample_query_lidar_configuration(std::string lidar_ip, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    zvision::DeviceConfigurationInfo info;
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		ret = config.QueryML30sPlusDeviceConfigurationInfo(info);
	}
	else {
		ret = config.QueryDeviceConfigurationInfo(info);
	}
    if (ret)
        LOG_F(ERROR, "Query device [%s]'s configuration info failed, ret = %d.", lidar_ip.c_str(), ret);
    else
    {
        std::string info_str = zvision::get_cfg_info_string(info);
        LOG_F(INFO, "Query device [%s]'s configuration info ok.", lidar_ip.c_str());
        LOG_F(INFO, "%s", info_str.c_str());
    }
    return ret;
}

//Sample code 11 : Get lidar's calibration file by tcp connection.
int sample_get_lidar_calibration(std::string lidar_ip, std::string savefilename, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.GetDeviceCalibrationDataToFile(savefilename, tp))
        LOG_F(ERROR, "Get device [%s]'s calibration data failed, ret = %d.", lidar_ip.c_str(), ret);
    else
    {
        LOG_F(INFO, "Get device [%s]'s calibration data ok.", lidar_ip.c_str());
        LOG_F(INFO, "Calibration data save to file %s.", savefilename.c_str());
    }
    return ret;
}

//Sample code 12 : Firmware update.
int sample_firmware_update(std::string lidar_ip, std::string filename, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		ret = config.ML30sPlusFirmwareUpdate(filename, print_current_progress_multi);
	}
	else {
		ret = config.FirmwareUpdate(filename, print_current_progress_multi);
	}
    if (ret)
        LOG_F(ERROR, "Update device [%s]'s firmware %s failed, ret = %d.", lidar_ip.c_str(), filename.c_str(), ret);
    else
    {
        LOG_F(INFO, "Update device [%s] fireware %s ok.", lidar_ip.c_str(), filename.c_str());
    }
    return ret;
}

//Sample code 13 : Reboot lidar by tcp connection.
int sample_reboot_lidar(std::string lidar_ip, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::reboot, &param_30sp);
	}
	else {
		ret = config.RebootDevice();
	}
    if (ret)
        LOG_F(ERROR, "Reboot device [%s] failed, ret = %d.", lidar_ip.c_str(), ret);
    else
    {
        LOG_F(INFO, "Reboot device [%s] ok.", lidar_ip.c_str());
    }
    return ret;
}


//Sample code 14 : Scan lidar on the heart beat port
//Notice, this function is supported by the lidar's new firmware kernel version, at least 0.1.20
int sample_scan_lidar_on_heat_beat_port(int seconds)
{
    int ret = 0;
    std::vector<zvision::DeviceConfigurationInfo> devices;
    
    if (ret = zvision::LidarTools::ScanDevice(devices, seconds))
        LOG_F(ERROR, "Scan device on heart beat port failed, ret = %d.", ret);
    else
    {
        LOG_F(INFO, "Scan device on heart beat port ok, total %d found.", devices.size());

        for (int i = 0; i < devices.size(); ++i)
        {
            zvision::DeviceConfigurationInfo& info = devices[i];
            std::string cfg_desp = zvision::get_cfg_info_string(info);
            LOG_F(INFO, "####################### Device number %3d #######################", i);
            LOG_F(INFO, "%s", cfg_desp.c_str());
            LOG_F(INFO, "#################################################################\n");
        }
    }
    return ret;
}

//Sample code 15 : Config lidar retro parameter 1(min ref, [0,100])
int sample_config_lidar_retro_param_min_ref(std::string lidar_ip, int ref)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceRetroParam1MinRef(ref))
        LOG_F(ERROR, "Set device [%s]'s retro param 1 to [%d] failed, ret = %d.", lidar_ip.c_str(), ref, ret);
    else
        LOG_F(INFO, "Set device [%s]'s retro param 1 to [%d] ok.", lidar_ip.c_str(), ref);
    return ret;
}

//Sample code 16 : Config lidar retro parameter 2(point percentage, [0,100])
int sample_config_lidar_retro_param_point_percentage(std::string lidar_ip, int percent)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceRetroParam2PointPercentage(percent))
        LOG_F(ERROR, "Set device [%s]'s retro param 2 to [%d] failed, ret = %d.", lidar_ip.c_str(), percent, ret);
    else
        LOG_F(INFO, "Set device [%s]'s retro param 2 to [%d] ok.", lidar_ip.c_str(), percent);
    return ret;
}

//Sample code 17 : Config lidar phaseoffset enable
int sample_config_lidar_phase_offset_enable(std::string lidar_ip, bool en, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.frame_sync = en;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_frame_sync, &param_30sp);
	}
	else {
		ret = config.SetDevicePhaseOffsetEnable(en);
	}
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s phase offset enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
    else
        LOG_F(INFO, "Set device [%s]'s phase offset enable to [%d] ok.", lidar_ip.c_str(), en);
    return ret;
}

//Sample code 18 : Config lidar phaseoffset value
int sample_config_lidar_phase_offset_value(std::string lidar_ip, int value_5ns, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.frame_offset = value_5ns;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_frame_offset, &param_30sp);
	}
	else {
		ret = config.SetDevicePhaseOffset(value_5ns);
	}
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s phase offset value to [%d]x5ns failed, ret = %d.", lidar_ip.c_str(), value_5ns, ret);
    else
        LOG_F(INFO, "Set device [%s]'s phase offset value to [%d]x5ns ok.", lidar_ip.c_str(), value_5ns);
    return ret;
}

//Sample code 19 : Config lidar ptp configuration file
int sample_config_lidar_ptp_configuration_file(std::string lidar_ip, std::string filename, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.temp_filepath = filename;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_ptp_file, &param_30sp);
	}
	else {
		ret = config.SetDevicePtpConfiguration(filename);
	}
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s ptp configuration file to [%s] failed, ret = %d.", lidar_ip.c_str(), filename.c_str(), ret);
    else
        LOG_F(INFO, "Set device [%s]'s ptp configuration file to [%s] ok.", lidar_ip.c_str(), filename.c_str());
    return ret;
}

//Sample code 20 : Get lidar ptp configuration file
int sample_get_lidar_ptp_configuration_to_file(std::string lidar_ip, std::string filename, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.temp_filepath = filename;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::read_ptp_file, &param_30sp);
	}
	else {
		ret = config.GetDevicePtpConfigurationToFile(filename);
	}
    if (ret)
        LOG_F(ERROR, "Get device [%s]'s ptp configuration file to [%s] failed, ret = %d.", lidar_ip.c_str(), filename.c_str(), ret);
    else
        LOG_F(INFO, "Get device [%s]'s ptp configuration file to [%s] ok.", lidar_ip.c_str(), filename.c_str());
    return ret;
}

//Sample code 21 : Config lidar calibration file broadcast enable
int sample_config_lidar_cali_file_broadcast_mode(std::string lidar_ip, bool en, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::CalSendMode mode = zvision::CalSendMode::CalSendDisable;
	if (en)
		mode = zvision::CalSendMode::CalSendEnable;

	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.angle_send = en;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_angle_send, &param_30sp);
	}
	else {
		ret = config.SetDeviceCalSendMode(mode);
	}
	if (ret)
		LOG_F(ERROR, "Set device [%s]'s calibration file broadcast enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
	else
		LOG_F(INFO, "Set device [%s]'s calibration file broadcast enable to [%d] ok.", lidar_ip.c_str(), en);
	return ret;
}

//Sample code 22 : Config lidar downsample mode
int sample_config_lidar_downsample_mode(std::string lidar_ip, std::string mode)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::DownsampleMode dm = zvision::DownsampleMode::DownsampleUnknown;
	if (mode.compare("none") == 0)
		dm = zvision::DownsampleNone;
	else if (mode.compare("1/2") == 0)
		dm = zvision::Downsample_1_2;
	else if (mode.compare("1/4") == 0)
		dm = zvision::Downsample_1_4;
	else {
		LOG_F(ERROR, "Set device [%s]'s downsample mode to [%s] failed, invalid parameters input.", lidar_ip.c_str(), mode.c_str());
		return ret;
	}

	if (ret = config.SetDeviceDownsampleMode(dm))
		LOG_F(ERROR, "Set device [%s]'s downsample mode to [%s] failed, ret = %d.", lidar_ip.c_str(), mode.c_str(), ret);
	else
		LOG_F(INFO, "Set device [%s]'s downsample mode to [%s] ok.", lidar_ip.c_str(), mode.c_str());
	return ret;
}

//Sample code 23 : Config lidar retro parameter
int sample_set_lidar_retro_parameters(std::string lidar_ip,int id, std::string val, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {

	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::DeviceAlgoParam param;
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		switch (id)
		{
		case 2:
			param_30sp.critical_point_dis_thre_retro = std::atoi(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_retro_critical_point_dis_thre, &param_30sp);
			break;
		case 3:
			LOG_F(ERROR, "Not support for 30sp device [%s].", lidar_ip.c_str());
			return -1;
		case 4:
			LOG_F(ERROR, "Not support for 30sp device [%s].", lidar_ip.c_str());
			return -1;
		case 1:
			param_30sp.target_point_num_thre_retro = (unsigned char)std::atoi(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_retro_target_point_num_thre, &param_30sp);
			break;
		case 5:
			param_30sp.del_point_gray_thre_retro = (unsigned char)std::atoi(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_retro_del_point_gray_thre, &param_30sp);
			break;
		case 6:
			param_30sp.del_point_dis_low_thre_retro = (unsigned char)std::atoi(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_retro_del_point_dis_low_thre, &param_30sp);
			break;
		case 7:
			param_30sp.del_point_dis_high_thre_retro = (unsigned char)std::atoi(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_retro_del_point_dis_high_thre, &param_30sp);
			break;
		case 8:
			param_30sp.target_gray_thre_retro = (unsigned char)std::atoi(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_retro_target_gray_thre, &param_30sp);
			break;
		default:
			ret = -1; break;
		}
	}
	else {
		switch (id)
		{
		case 2:
			ret = config.SetDeviceRetroParam(zvision::RetroDisThres,std::atoi(val.c_str()));break;
		case 3:
			ret = config.SetDeviceRetroParam(zvision::RetroLowRangeThres, (unsigned short)std::atoi(val.c_str())); break;
		case 4:
			ret = config.SetDeviceRetroParam(zvision::RetroHighRangeThres, (unsigned short)std::atoi(val.c_str())); break;
		case 1:
			ret = config.SetDeviceRetroParam(zvision::RetroMinGrayNum, (unsigned char)std::atoi(val.c_str())); break;
		case 5:
			ret = config.SetDeviceRetroParam(zvision::RetroDelGrayThres, (unsigned char)std::atoi(val.c_str())); break;
		case 6:
			ret = config.SetDeviceRetroParam(zvision::RetroDelRatioGrayLowThres, (unsigned char)std::atoi(val.c_str())); break;
		case 7:
			ret = config.SetDeviceRetroParam(zvision::RetroDelRatioGrayHighThres, (unsigned char)std::atoi(val.c_str())); break;
		case 8:
			ret = config.SetDeviceRetroParam(zvision::RetroMinGray, (unsigned char)std::atoi(val.c_str())); break;
		default:
			ret = -1; break;
		}
	}
	

	if (ret != 0)
		LOG_F(ERROR, "Set device [%s]'s retro parameters failed, ret = %d.", lidar_ip.c_str(), ret);
	else
		LOG_F(INFO, "Set device [%s]'s retro parameters ok.", lidar_ip.c_str());

	return ret;
}

//Sample code 24 : Config lidar adhesion parameter
int sample_set_lidar_adhesion_parameters(std::string lidar_ip, int id, std::string val, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::DeviceAlgoParam param;
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		switch (id)
		{
		case 1:
			param_30sp.angle_hor_min_adhesion = std::atoi(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_adhesion_angle_hor_min, &param_30sp);
			break;
		case 2:
			param_30sp.angle_hor_max_adhesion = std::atoi(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_adhesion_angle_hor_max, &param_30sp);
			break;
		case 3:
			param_30sp.angle_ver_min_adhesion = std::atoi(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_adhesion_angle_ver_min, &param_30sp);
			break;
		case 4:
			param_30sp.angle_ver_max_adhesion = std::atoi(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_adhesion_angle_ver_max, &param_30sp);
			break;
		case 5:
			param_30sp.angle_hor_res_adhesion = (float)std::atof(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_adhesion_angle_hor_res, &param_30sp);
			break;
		case 6:
			param_30sp.angle_ver_res_adhesion = (float)std::atof(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_adhesion_angle_ver_res, &param_30sp);
			break;
		case 7:
			param_30sp.diff_thre_adhesion = (float)std::atof(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_adhesion_diff_thre, &param_30sp);
			break;
		case 8:
			param_30sp.dist_limit_adhesion = (float)std::atof(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_adhesion_dist_limit, &param_30sp);
			break;
		case 9:
			param_30sp.min_diff_adhesion = (float)std::atof(val.c_str());
			ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_adhesion_min_diff, &param_30sp);
			break;
		default:
			ret = -1; break;
		}
	}
	else {
		switch (id)
		{
		case 1:
			ret = config.SetDeviceAdhesionParam(zvision::MinimumHorizontalAngleRange, std::atoi(val.c_str())); break;
		case 2:
			ret = config.SetDeviceAdhesionParam(zvision::MaximumHorizontalAngleRange, std::atoi(val.c_str())); break;
		case 3:
			ret = config.SetDeviceAdhesionParam(zvision::MinimumVerticalAngleRange, std::atoi(val.c_str())); break;
		case 4:
			ret = config.SetDeviceAdhesionParam(zvision::MaximumVerticalAngleRange, std::atoi(val.c_str())); break;
		case 5:
			ret = config.SetDeviceAdhesionParam(zvision::HorizontalAngleResolution, (float)std::atof(val.c_str())); break;
		case 6:
			ret = config.SetDeviceAdhesionParam(zvision::VerticalAngleResolution, (float)std::atof(val.c_str())); break;
		case 7:
			ret = config.SetDeviceAdhesionParam(zvision::DeletePointThreshold, (float)std::atof(val.c_str())); break;
		case 8:
			ret = config.SetDeviceAdhesionParam(zvision::MaximumProcessingRange, (float)std::atof(val.c_str())); break;
		case 9:
			ret = config.SetDeviceAdhesionParam(zvision::NearFarPointDiff, (float)std::atof(val.c_str())); break;
		default:
			ret = -1; break;
		}
	}

	if (ret != 0)
		LOG_F(ERROR, "Set device [%s]'s adhesion parameters failed, ret = %d.", lidar_ip.c_str(), ret);
	else
		LOG_F(INFO, "Set device [%s]'s adhesion parameters ok.", lidar_ip.c_str());

	return ret;
}

//Sample code 25 : Get lidar algorithm parameter(retro and adhesion)
int sample_get_lidar_algorithm_parameters(std::string lidar_ip, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {

	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::DeviceAlgoParam param;
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::read_algo_param_in_use, &param_30sp);
		if (ret == 0) {
			// retro
			param.retro_min_gray_num = param_30sp.target_point_num_thre_retro;
			param.retro_dis_thres = param_30sp.critical_point_dis_thre_retro;
			param.retro_del_gray_thres = param_30sp.del_point_gray_thre_retro;
			param.retro_del_ratio_gray_low_thres = param_30sp.del_point_dis_low_thre_retro;
			param.retro_del_ratio_gray_high_thres = param_30sp.del_point_dis_high_thre_retro;
			param.retro_min_gray = param_30sp.target_gray_thre_retro;
			// adhesion
			param.adhesion_angle_hor_min = param_30sp.angle_hor_min_adhesion;
			param.adhesion_angle_hor_max = param_30sp.angle_hor_max_adhesion;
			param.adhesion_angle_ver_min = param_30sp.angle_ver_min_adhesion;
			param.adhesion_angle_ver_max = param_30sp.angle_ver_max_adhesion;
			param.adhesion_angle_hor_res = param_30sp.angle_hor_res_adhesion;
			param.adhesion_angle_ver_res = param_30sp.angle_ver_res_adhesion;
			param.adhesion_diff_thres = param_30sp.diff_thre_adhesion;
			param.adhesion_dis_limit = param_30sp.dist_limit_adhesion;
			param.adhesion_min_diff = param_30sp.min_diff_adhesion;
		}
	}
	else {
		ret = config.QueryDeviceAlgoParam(param);
	}

	if (ret)
		LOG_F(ERROR, "Get device [%s]'s algorithm parameters failed, ret = %d.", lidar_ip.c_str(), ret);
	else {
		LOG_F(INFO, "Get device [%s]'s algorithm parameters ok.", lidar_ip.c_str());
		std::string msg;
		msg += "------  Retro  -------\n";
		msg += "min gray num: " + std::to_string(param.retro_min_gray_num) + "\n";
		msg += "dis thres: " + std::to_string(param.retro_dis_thres) + "\n";
		if (tp != zvision::DeviceType::LidarMl30SA1Plus) {
			msg += "low range thres: " + std::to_string(param.retro_low_range_thres) + "\n";
			msg += "high range thres: " + std::to_string(param.retro_high_range_thres) + "\n";
		}
		msg += "del gray thres: " + std::to_string(param.retro_del_gray_thres) + "\n";
		msg += "del ratio gray low thres: " + std::to_string(param.retro_del_ratio_gray_low_thres) + "\n";
		msg += "del ratio gray high thres: " + std::to_string(param.retro_del_ratio_gray_high_thres) + "\n";
		msg += "min gray: " + std::to_string(param.retro_min_gray) + "\n";
		msg += "-----  Adhesion  -----\n";
		msg += "angle hor min: " + std::to_string(param.adhesion_angle_hor_min) +"\n";
		msg += "angle hor max: " + std::to_string(param.adhesion_angle_hor_max) +"\n";
		msg += "angle ver min: " + std::to_string(param.adhesion_angle_ver_min) +"\n";
		msg += "angle ver max: " + std::to_string(param.adhesion_angle_ver_max) +"\n";
		msg += "angle hor res: " + std::to_string(param.adhesion_angle_hor_res) +"\n";
		msg += "angle ver res: " + std::to_string(param.adhesion_angle_ver_res) +"\n";
		msg += "diff thres: " + std::to_string(param.adhesion_diff_thres) + "\n";
		msg += "dis limit: " + std::to_string(param.adhesion_dis_limit) + "\n";
		msg += "min diff: " + std::to_string(param.adhesion_min_diff) + "\n";
		LOG_F(INFO, msg.c_str());
	}

	return ret;
}

//Sample code 26 : Config lidar delete close points enable
int sample_config_lidar_delete_points(std::string lidar_ip, bool en, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.switch_near_point_delete = en;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_near_point_delete_switch, &param_30sp);
	}
	else {
		ret = config.SetDeviceAlgorithmEnable(zvision::AlgoType::AlgoDeleteClosePoints, en);
	}
	if (ret)
		LOG_F(ERROR, "Set device [%s]'s delete close points enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
	else
		LOG_F(INFO, "Set device [%s]'s delete close points enable to [%d] ok.", lidar_ip.c_str(), en);
	return ret;
}

//Sample code 27 : Config lidar adhesion enable
int sample_config_lidar_adhesion(std::string lidar_ip, bool en, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.switch_adhesion = en;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_near_point_delete_switch, &param_30sp);
	}
	else {
		ret = config.SetDeviceAlgorithmEnable(zvision::AlgoType::AlgoAdhesion, en);
	}
	if (ret)
		LOG_F(ERROR, "Set device [%s]'s adhesion enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
	else
		LOG_F(INFO, "Set device [%s]'s adhesion enable to [%d] ok.", lidar_ip.c_str(), en);
	return ret;
}

/* Handling multiple lidars */
//Sample code 28 : Muitiple lidars Firmware update.
int sample_firmware_update_multi(std::vector<std::string> lidars_ip, std::string filename, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
	// start task
	int devs = lidars_ip.size();
	std::vector<std::shared_ptr<std::thread>> thr_vec;
	thr_vec.resize(devs);
	for (int i = 0; i < devs; i++) {
		thr_vec[i].reset(new std::thread(std::bind(&sample_firmware_update, lidars_ip[i], filename, tp)));
	}
	// wait for finish
	for (int i = 0; i < devs; i++) {
		if (thr_vec[i]->joinable())
			thr_vec[i]->join();
	}
	return 0;
}
//Sample code 29 : Config muitiple lidars adhesion enable.
int sample_config_lidar_adhesion_multi(std::vector<std::string> lidars_ip, bool en, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {

	// start task
	for (auto ip : lidars_ip)
		sample_config_lidar_adhesion(ip, en, tp);

	return 0;
}
//Sample code 30 : Config muitiple lidars delete_points enable.
int sample_config_lidar_delete_points_multi(std::vector<std::string> lidars_ip, bool en, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {

	// start task
	for (auto ip : lidars_ip)
		sample_config_lidar_delete_points(ip, en, tp);

	return 0;
}
//Sample code 31 : Config muitiple lidars retro enable.
int sample_config_lidar_retro_multi(std::vector<std::string> lidars_ip, bool en, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {

	// start task
	for (auto ip: lidars_ip)
		sample_config_lidar_retro_enable(ip, en, tp);

	return 0;
}
//Sample code 32 : Config muitiple lidars phase offset enable.
int sample_config_lidar_phase_offset_multi(std::vector<std::string> lidars_ip, bool en, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {

	// start task
	for (auto ip : lidars_ip)
		sample_config_lidar_phase_offset_enable(ip, en, tp);

	return 0;
}
//Sample code 33 : Config muitiple lidars phase offset enable.
int sample_query_lidar_configuration_multi(std::vector<std::string> lidars_ip, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {

	// start task
	for (auto ip: lidars_ip)
		sample_query_lidar_configuration(ip, tp);

	return 0;
}
//Sample code 34 : Reboot lidars by tcp connection.
int sample_reboot_lidar_multi(std::vector<std::string> lidars_ip, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {

	// start task
	for (auto ip: lidars_ip)
		sample_reboot_lidar(ip, tp);

	return 0;
}

//Sample code 35 : Set lidar's calibration file by tcp connection.
int sample_config_lidar_calibration(std::string lidar_ip, std::string filename, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown) {
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (ret = config.SetDeviceCalibrationData(filename, tp))
		LOG_F(ERROR, "Set device [%s]`s calibration data failed, ret = %d.", lidar_ip.c_str(), ret);
	else
	{
		LOG_F(INFO, "Set device [%s]`s calibration data ok.", lidar_ip.c_str());
	}
	return ret;
}

// Sample code 36 : Get lidar's channel file by tcp connection.
int sample_get_lidar_channel_file(std::string lidar_ip, std::string savefilename, bool curr = true)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (ret = config.GetDeviceChannelDataToFile(savefilename, curr))
		LOG_F(ERROR, "Get device [%s]'s channel data failed, ret = %d.", lidar_ip.c_str(), ret);
	else
	{
		LOG_F(INFO, "Get device [%s]'s channel data ok.", lidar_ip.c_str());
		LOG_F(INFO, "Channel data save to file %s.", savefilename.c_str());
	}
	return ret;
}

// Sample code 37 : Get lidar's channel list file by tcp connection.
int sample_get_lidar_channel_list_file(std::string lidar_ip, std::string savefilename)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (ret = config.GetDeviceChannelListDataToFile(savefilename))
		LOG_F(ERROR, "Get device [%s]'s channel list data failed, ret = %d.", lidar_ip.c_str(), ret);
	else
	{
		LOG_F(INFO, "Get device [%s]'s channel list data ok.", lidar_ip.c_str());
		LOG_F(INFO, "Channel list data save to file %s.", savefilename.c_str());
	}
	return ret;
}

// Sample code 38 : Set lidar's echo mode.
int sample_config_lidar_echo_mode(std::string lidar_ip, int id, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
	zvision::EchoMode mode = zvision::EchoMode::EchoUnknown;
	if (id == 1)
		mode = zvision::EchoSingleFirst;
	else if (id == 2)
		mode = zvision::EchoSingleStrongest;
	else if (id == 3)
		mode = zvision::EchoSingleLast;
	else if (id == 4)
		mode = zvision::EchoDoubleFirstStrongest;
	else if (id == 5)
		mode = zvision::EchoDoubleFirstLast;
	else if (id == 6)
		mode = zvision::EchoDoubleStrongestLast;
	else {
		LOG_F(ERROR, "Invalid echo mode parameters [%d] ,set lidat echo mode failed..", id);
		return zvision::InvalidParameter;
	}

	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int ret = 0;
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		param_30sp.echo_mode = mode;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::set_echo_mode, &param_30sp);
	}
	else {
		ret = config.SetDeviceEchoMode(mode);
	}

	if (ret == zvision::NotSupport)
		LOG_F(INFO, "Set device [%s]'s echo mode to [%s] ignored, not support.", lidar_ip.c_str(), zvision::get_echo_mode_string(mode).c_str(), ret);
	else if(ret)
		LOG_F(ERROR, "Set device [%s]'s echo mode to [%s] failed, ret = %d.", lidar_ip.c_str(), zvision::get_echo_mode_string(mode).c_str(), ret);
	else
	{
		LOG_F(INFO, "Set device [%s]'s echo mode to [%s] ok.", lidar_ip.c_str(), zvision::get_echo_mode_string(mode).c_str());
	}
	return ret;
}

// Sample code 39 : Get lidar's train log.
int sample_get_lidar_train_log(std::string lidar_ip, std::string path) {
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	std::string context;
	ret = config.GetDeviceLog(context);
	if (ret)
	{
		LOG_F(ERROR, "Get device [%s]`s log failed, ret = %d.", lidar_ip.c_str(), ret);
		return -1;
	}
	else
	{
		LOG_F(INFO, "Get device [%s]`s log ok.", lidar_ip.c_str());
	}

	// get log line data
	std::vector<std::string> lines;
	strSplit(context, lines,"\n");

	// open file
	std::fstream outfile;
	outfile.open(path, std::ios::out);
	if (!outfile.is_open()) {
		LOG_F(ERROR, "Can not open log file [%s].", path.c_str());
		return -1;
	}

	// save to file
	std::string tag = "[TRAIN]WRITE FLASH";
	for (auto& line:lines) {
		size_t pos = line.find(tag);
		if (pos == line.npos)
			continue;

		outfile << line.substr(pos, line.size() - pos) << std::endl;
	}
	outfile.close();
	LOG_F(INFO, "Save device [%s]`s train log file to [%s].", lidar_ip.c_str(), path.c_str());

	return 0;
}

// Sample code 40 : Get lidar's log file.
int sample_get_lidar_log(std::string lidar_ip, std::string path, zvision::DeviceType tp = zvision::DeviceType::LidarUnknown)
{
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	std::string context;
	int ret = 0;
	if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
		zvision::JsonConfigFileParam param_30sp;
		ret = config.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::read_full_log, &param_30sp);
		if (ret == 0)
		{
			context = param_30sp.temp_recv_data;
		}
	}
	else
	{
		ret = config.GetDeviceLog(context);
	}

	if (ret)
	{
		LOG_F(ERROR, "Get device [%s]`s log failed, ret = %d.", lidar_ip.c_str(), ret);
		return -1;
	}
	else
	{
		LOG_F(INFO, "Get device [%s]`s log ok.", lidar_ip.c_str());
	}

	// open file
	std::fstream outfile;
	outfile.open(path, std::ios::out);
	if (!outfile.is_open()) {
		LOG_F(ERROR, "Can not open log file [%s].", path.c_str());
		return -1;
	}
	// save to file
	outfile << context;
	outfile.close();
	LOG_F(INFO, "Save device [%s]`s log file to [%s].", lidar_ip.c_str(), path.c_str());

	return 0;
}

// Sample code 41 : Set lidar's cover file.
int sample_set_lidar_cover_file(std::string lidar_ip, std::string path, int itp = 1)
{
	zvision::DeviceType tp = zvision::DeviceType::LidarML30SB2;
	if (itp == 0) {
		tp = zvision::DeviceType::LidarML30B1;
	}

	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (tp == zvision::DeviceType::LidarMl30SA1Plus || tp == zvision::DeviceType::LidarMl30SB1Plus)
	{
		LOG_F(ERROR, "ML30S+ series lidar is not supported temporarily.");
		return -1;
	}

	ret = config.SetDevicePointFireEnConfiguration(path, tp);
	if (ret)
	{
		LOG_F(ERROR, "Set lidar [%s]'s cover file failed, return code %d.", lidar_ip.c_str(), ret);
		return -1;
	}
	else
	{
		LOG_F(INFO, "Set lidar [%s]'s cover file ok.", lidar_ip.c_str());
	}

	return 0;
}

// Sample code 42 :  update lidar  adc algorithm parameter
	/** \brief update lidar  adc algorithm parameter.
	* \param[in] lidar_ip           the device ip address.
	* \param[in] input_dir       input files directory.
	* \param[in] reset              if reset is true download origion parameter file or regenerate parameter file.
	* \param[in] output_dir   if reset is false and output_dir is not empty, the new parameter file will be saved into output_dir.
	* \return 0 for ok, others for failure.
	*/
int sample_adc_algo_param_update(std::string lidar_ip, std::string input_dir, bool reset, std::string output_dir = "")
{
	// initialize connection
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);

	// 1. get lidar serial number
	std::string serial_number;
	ret = config.QueryDeviceSnCode(serial_number);
	if (ret)
	{
		LOG_F(ERROR, "Query device [%s]'s serial number failed, ret = %d.", lidar_ip.c_str(), ret);
		return ret;
	}
	else
	{
		LOG_F(INFO, "Query device [%s]'s serial number[%s] ok.", lidar_ip.c_str(), serial_number.c_str());
	}

	// 2. get local adc algorithm parameter filepaths
	std::string root_dir = input_dir;
	if (root_dir.back() == '\\' || root_dir.back() == '/')
		root_dir = root_dir.substr(0, root_dir.size() - 1);

	// set default path`s sub directory name
	std::string file_dir;
	if (!reset)
		file_dir = root_dir + "/30S" + serial_number + "/Parameter/";
	else
		file_dir = root_dir + "/" + serial_number + "/";

	std::vector<std::string> files_name;
	getFileListInDir(file_dir, files_name);
	if (files_name.size() == 0)
	{
		LOG_F(ERROR, "Get device [%s]'s adc algorithm parameter files from[%s] failed.", lidar_ip.c_str(), file_dir.c_str());
		return -1;
	}

	// 3. check files name
	if (ret = checkAdcAlgoParamFilesName(files_name, serial_number, reset))
	{
		LOG_F(ERROR, "Check device [%s]'s adc algorithm parameter files from[%s] failed.", lidar_ip.c_str(), file_dir.c_str());
		return ret;
	}

	// 4. get adc algorithm parameter flash data
	std::string flash_data;
	if (!reset)
	{
		//  4.1 get lidar adc algorithm parameter flash data
		ret = config.GetDeviceFlashConfiguration(flash_data, zvision::FlashParamType::ADC_Algo);
		if (ret)
		{
			LOG_F(ERROR, "Get device [%s]'s adc algorithm parameter data failed.", lidar_ip.c_str());
			return ret;
		}
		else
			LOG_F(INFO, "Get device [%s]'s adc algorithm parameter data ok.", lidar_ip.c_str());

		//  4.2 regenerate new adc algorithm parameter flash data
		ret = regenerateAdcAlgoParamFlashData(file_dir, files_name, flash_data);
		if (ret)
		{
			LOG_F(ERROR, "Regenerate device [%s]'s adc algorithm parameter data failed.", lidar_ip.c_str());
			return ret;
		}
		else
			LOG_F(INFO, "Regenerate device [%s]'s adc algorithm parameter data ok.", lidar_ip.c_str());

		// 4.3 save adc algorithm parameter flash data to file
		if (!output_dir.empty())
		{
			// adc algorithm parameter file save path
			std::string file_path = output_dir;
			if (file_path.back() == '\\' || file_path.back() == '/')
				file_path = file_path.substr(0, file_path.size() - 1);

			file_path = file_path + "/" + serial_number + "_output.adc_all";
			ret = saveAdcAlgoParamFlashDataToLocal(flash_data, file_path);
			if (ret)
				LOG_F(WARNING, "Save [%s]'s new adc algorithm parameter data to[%s] failed.", lidar_ip.c_str(), file_path.c_str());
			else
				LOG_F(INFO, "Save [%s]'s new adc algorithm parameter data to[%s] ok.", lidar_ip.c_str(), file_path.c_str());
		}
	}
	else
	{
		// 4.1 read adc algorithm parameter flash data from file
		std::string file_path = file_dir + files_name[0];
		ret = readAdcDataFromFile(file_path, flash_data);
		if (ret)
		{
			LOG_F(ERROR, "Load device [%s]'s adc algorithm parameter file from[%s] failed.", lidar_ip.c_str(), file_path.c_str());
			return ret;
		}
	}

	// 5. download flash data to lidar
	ret = config.SetDeviceFlashConfiguration(flash_data, zvision::FlashParamType::ADC_Algo);
	if (ret)
	{
		LOG_F(ERROR, "Update device [%s]'s adc algorithm parameter data failed.", lidar_ip.c_str());
		return ret;
	}
	else
		LOG_F(INFO, "Update device [%s]'s adc algorithm parameter data ok.", lidar_ip.c_str());

	//  6. reboot lidar
	ret = config.RebootDevice();
	if (ret)
	{
		LOG_F(ERROR, "Reboot device [%s] failed, ret = %d.", lidar_ip.c_str(), ret);
		return ret;
	}
	else
		LOG_F(INFO, "Reboot device [%s] ok.", lidar_ip.c_str());

	// 7. Log adc algorithm parameter  information.
	std::stringstream ss;
	ss << "Summary:\n";
	ss << "  sn:" << serial_number << "\n";
	ss << "  file directory:" << file_dir << "\n";
	ss << "  files name:  " << files_name[0] << "\n";
	for (int i = 1; i < files_name.size(); ++i)
		ss << "              " << files_name[i] << "\n";

	LOG_F(INFO, ss.str().c_str());
	return ret;
}

//Sample code 43 : update lidars  adc algorithm parameter
int sample_adc_algo_param_update_multi(std::vector<std::string> lidars_ip, std::string input_dir, bool reset, std::string output_dir = "") {

	for (auto ip : lidars_ip)
		sample_adc_algo_param_update(ip, input_dir, reset, output_dir);

	return 0;
}

// Sample code 44 :  read lidar  adc algorithm parameter
	/** \brief read lidar  adc algorithm parameter.
	* \param[in] lidar_ip           the device ip address.
	* \param[in] filepath           adc algorithm parameter file path.
	* \return 0 for ok, others for failure.
	*/
int sample_get_adc_algo_param(std::string lidar_ip, std::string filepath)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	std::string flash_data;
	ret = config.GetDeviceFlashConfiguration(flash_data, zvision::FlashParamType::ADC_Algo);
	if (ret)
	{
		LOG_F(ERROR, "Get device [%s]'s  adc algorithm parameter data failed.", lidar_ip.c_str());
		return ret;
	}

	ret = saveAdcAlgoParamFlashDataToLocal(flash_data, filepath);
	if (ret)
	{
		LOG_F(ERROR, "Save [%s]'s adc algorithm parameter data to[%s] failed.", lidar_ip.c_str(), filepath.c_str());
	}
	else
		LOG_F(INFO, "Save [%s]'s adc algorithm parameter data to[%s] ok.", lidar_ip.c_str(), filepath.c_str());

	return ret;
}

// Sample code 45 :  config lidar layer detection
int sample_config_lidar_layer_detection(std::string lidar_ip, bool en)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	ret = config.SetDeviceLayerDetectionEnable(en);
	if (ret)
		LOG_F(ERROR, "Set device [%s]'s layer detection enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
	else
		LOG_F(INFO, "Set device [%s]'s layer detection enable to [%d] ok.", lidar_ip.c_str(), en);
	return ret;
}

// Sample code 46 :  config lidar PL delete near point level
int sample_config_lidar_pl_delete_near_point_level(std::string lidar_ip, int mode)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	ret = config.SetDeviceDeleteNearPointLevel(mode);
	if (ret)
		LOG_F(ERROR, "Set device [%s]'s delete near point level to [%d] failed, ret = %d.", lidar_ip.c_str(), mode, ret);
	else
		LOG_F(INFO, "Set device [%s]'s delete near point level to [%d] ok.", lidar_ip.c_str(), mode);
	return ret;
}

void init_log(int argc, char* argv[])
{
	loguru::init();
	loguru::add_file("Log/log.txt", loguru::Truncate, loguru::Verbosity_INFO);
	loguru::g_stderr_verbosity = 4;
	loguru::g_preamble_date = 1;
	loguru::g_preamble_time = 1;
	loguru::g_preamble_uptime = 1;
	loguru::g_preamble_thread = 1;
	loguru::g_preamble_file = 1;
	loguru::g_preamble_verbose = 1;
	loguru::g_preamble_pipe = 1;
}

int main(int argc, char** argv)
{
    if (argc <= 2)
    {
        std::cout << "############################# USER GUIDE ################################\n\n"

			<< "Note: For ML30S+ device, you can add \"-30sp\" follow the command flag\n\n"

            << "Sample 0 : config mac address\n"
            << "Format: -config_mac lidar_ip mac_address\n"
			<< "Format: -config_mac -30sp lidar_ip mac_address\n"
            << "Demo:   -config_mac 192.168.10.108 66-66-66-66-66-66\n"
			<< "Demo:   -config_mac -30sp 192.168.10.108 66-66-66-66-66-66\n\n"

            << "Sample 1 : config static ip address\n"
            << "Format: -config_static_ip old_ip new_ip\n"
			<< "Format: -config_static_ip -30sp old_ip new_ip\n"
            << "Demo:   -config_static_ip 192.168.10.108 192.168.10.107\n"
			<< "Demo:   -config_static_ip -30sp 192.168.10.108 192.168.10.107\n\n"

            << "Sample 2 : config subnet mask\n"
            << "Format: -config_subnet_mask lidar_ip subnet_mask\n"
			<< "Format: -config_subnet_mask -30sp lidar_ip subnet_mask\n"
            << "Demo:   -config_subnet_mask 192.168.10.108 255.255.255.0\n"
			<< "Demo:   -config_subnet_mask -30sp 192.168.10.108 255.255.255.0\n\n"


            << "Sample 3 : config udp destination ip address\n"
            << "Format: -config_dst_ip lidar_ip dst_ip\n"
			<< "Format: -config_dst_ip -30sp lidar_ip dst_ip\n"
			<< "Demo:   -config_dst_ip 192.168.10.108 192.168.10.255\n"
            << "Demo:   -config_dst_ip -30sp 192.168.10.108 192.168.10.255\n\n"

            << "Sample 4 : config udp destination port\n"
            << "Format: -config_dst_port lidar_ip port\n"
			<< "Format: -config_dst_port -30sp lidar_ip port\n"
			<< "Demo:   -config_dst_port 192.168.10.108 2368\n"
            << "Demo:   -config_dst_port -30sp 192.168.10.108 2368\n\n"

            << "Sample 5 : config retro mode\n"
            << "Format: -config_retro lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_retro_multi lidar1_ip lidar2_ip (MaxLidarCount:4) mode\n"
			<< "Format: -config_retro -30sp lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_retro_multi -30sp lidar1_ip lidar2_ip (MaxLidarCount:4) mode\n"
            << "Demo:   -config_retro 192.168.10.108 0\n"
			<< "Demo:   -config_retro_multi 192.168.10.108 192.168.10.109 0\n"
			<< "Demo:   -config_retro -30sp 192.168.10.108 0\n"
			<< "Demo:   -config_retro_multi -30sp 192.168.10.108 192.168.10.109 0\n\n"

            << "Sample 6 : config time sync mode\n"
            << "Format: -config_time_sync lidar_ip mode(0 for ptp, 1 for gpspps)\n"
            << "Demo:   -config_time_sync 192.168.10.108 0\n\n"

            << "Sample 7 : query firmware version\n"
            << "Format: -query_version lidar_ip\n"
			<< "Format: -query_version -30sp lidar_ip\n"
			<< "Demo:   -query_version 192.168.10.108\n"
            << "Demo:   -query_version -30sp 192.168.10.108\n\n"

            << "Sample 8 : query serial number\n"
            << "Format: -query_sn lidar_ip\n"
			<< "Format: -query_sn -30sp lidar_ip\n"
			<< "Demo:   -query_sn 192.168.10.108\n"
            << "Demo:   -query_sn -30sp 192.168.10.108\n\n"

            << "Sample 9 : query hardware temperature\n"
            << "Format: -query_temp lidar_ip\n"
            << "Demo:   -query_temp 192.168.10.108\n\n"

            << "Sample 10 : query configuration\n"
            << "Format: -query_cfg lidar_ip\n"
			<< "Format: -query_cfg_multi lidar1_ip lidar2_ip(MaxLidarCount:4)\n"
			<< "Format: -query_cfg -30sp lidar_ip\n"
			<< "Format: -query_cfg_multi -30sp lidar1_ip lidar2_ip(MaxLidarCount:4)\n"
            << "Demo:   -query_cfg 192.168.10.108\n"
			<< "Demo:   -query_cfg_multi 192.168.10.108 192.168.10.109\n"
			<< "Demo:   -query_cfg -30sp 192.168.10.108\n"
			<< "Demo:   -query_cfg_multi -30sp 192.168.10.108 192.168.10.109\n\n"

            << "Sample 11 : get calibration data to file\n"
            << "Format: -get_cal lidar_ip savefilename\n"
			<< "Format: -get_cal -30sp lidar_ip savefilename\n"
			<< "Demo:   -get_cal 192.168.10.108 device.cal\n"
            << "Demo:   -get_cal -30sp 192.168.10.108 device.cal\n\n"

            << "Sample 12 : firmware update\n"
            << "Format: -firmware_update lidar_ip filename\n"
			<< "Format: -firmware_update_multi lidar1_ip lidar2_ip(MaxLidarCount:4) filename\n"
			<< "Format: -firmware_update -30sp lidar_ip filename\n"
			<< "Format: -firmware_update_multi -30sp lidar1_ip lidar2_ip(MaxLidarCount:4) filename\n"
			<< "Demo:   -firmware_update 192.168.10.108 firmware_name.pack\n"
			<< "Demo:   -firmware_update_multi 192.168.10.108 192.168.10.109 firmware_name.pack\n"
            << "Demo:   -firmware_update -30sp 192.168.10.108 firmware_name.pack\n"
			<< "Demo:   -firmware_update_multi -30sp 192.168.10.108 192.168.10.109 firmware_name.pack\n\n"

            << "Sample 13 : reboot\n"
            << "Format: -reboot lidar_ip\n"
			<< "Format: -reboot_multi lidar1_ip lidar2_ip(MaxLidarCount:4) \n"
			<< "Format: -reboot -30sp lidar_ip\n"
			<< "Format: -reboot_multi -30sp lidar1_ip lidar2_ip(MaxLidarCount:4) \n"
			<< "Demo:   -reboot 192.168.10.108\n"
			<< "Demo:   -reboot_multi 192.168.10.108 192.168.10.109\n"
            << "Demo:   -reboot -30sp 192.168.10.108\n"
			<< "Demo:   -reboot_multi -30sp 192.168.10.108 192.168.10.109\n\n"

            << "Sample 14 : scan device\n"
            << "Format: -scan_device scan_time(s)\n"
            << "Demo:   -scan_device 5\n\n"

            << "Sample 15 : retro param 1(min ref[0,100])\n"
            << "Format: -retro_p1 value\n"
            << "Demo:   -retro_p1 5\n\n"

            << "Sample 16 : retro param 2(point percentage[0,100])\n"
            << "Format: -retro_p2 value\n"
            << "Demo:   -retro_p2 5\n\n"

            << "Sample 17 : phase offset enable(0 for disable, 1 for enable)\n"
            << "Format: -phase_offset_enable lidar_ip mode\n"
			<< "Format: -phase_offset_enable_multi lidar1_ip lidar2_ip(MaxLidarCount:4)\n"
			<< "Format: -phase_offset_enable -30sp lidar_ip mode\n"
			<< "Format: -phase_offset_enable_multi -30sp lidar1_ip lidar2_ip(MaxLidarCount:4)\n"
			<< "Demo:   -phase_offset_enable 192.168.10.108 0\n"
			<< "Demo:   -phase_offset_enable_multi 192.168.10.108 192.168.10.109 0\n"
            << "Demo:   -phase_offset_enable -30sp 192.168.10.108 0\n"
			<< "Demo:   -phase_offset_enable_multi -30sp 192.168.10.108 192.168.10.109 0\n\n"

            << "Sample 18 : phase offset value\n"
            << "Format: -phase_offset_value lidar_ip value(x5ns)\n"
			<< "Format: -phase_offset_value -30sp lidar_ip value(x5ns)\n"
			<< "Demo:   -phase_offset_value 192.168.10.108 0\n"
            << "Demo:   -phase_offset_value -30sp 192.168.10.108 0\n\n"

            << "Sample 19 : config ptp configuration\n"
            << "Format: -set_ptp_cfg lidar_ip filename\n"
			<< "Format: -set_ptp_cfg -30sp lidar_ip filename\n"
			<< "Demo:   -set_ptp_cfg 192.168.10.108 test.txt\n"
            << "Demo:   -set_ptp_cfg -30sp 192.168.10.108 test.txt\n\n"

            << "Sample 20 : get ptp configuration to file\n"
            << "Format: -get_ptp_cfg lidar_ip filename\n"
			<< "Format: -get_ptp_cfg -30sp lidar_ip filename\n"
			<< "Demo:   -get_ptp_cfg 192.168.10.108 test.txt\n"
            << "Demo:   -get_ptp_cfg -30sp 192.168.10.108 test.txt\n\n"

			<< "Sample 21 : calibration file broadcast enabale\n"
			<< "Format: -cali_file_broadcast_enable lidar_ip enable(0 for disable, 1 for enable) \n"
			<< "Format: -cali_file_broadcast_enable -30sp lidar_ip enable(0 for disable, 1 for enable) \n"
			<< "Demo:   -cali_file_broadcast_enable 192.168.10.108 0\n"
			<< "Demo:   -cali_file_broadcast_enable -30sp 192.168.10.108 0\n\n"

			<< "Sample 22 : config downsample mode\n"
			<< "Format: -downsample_mode lidar_ip mode( none: no downsample, 1/2: 50% downsample, 1/4: 25% downsample)\n"
			<< "Demo:   -downsample_mode 192.168.10.108 none\n\n"

			<< "Sample 23 : set Retro parameters\n"
			<< "1. min gray num:              High gray value num threshold  (int8)\n"
			<< "2. dis thres:                 Lock critical point distance   (uint32)\n"
			<< "3. low range thres:           Delete point range low threshold distance(not support ML30S+ device)  (uint16)\n"
			<< "4. high range thres:          Delete point range high threshold distance(not support ML30S+ device) (uint16)\n"
			<< "5. del gray thres::           Delete point grayscale threshold   (uint8)\n"
			<< "6. del ratio gray low thres:  The gray value low threshold of the deleted point ratio  (uint8)\n"
			<< "7. del ratio gray high thres: The gray value high threshold of the deleted point ratio (uint8)\n"
			<< "8. min gray:                  Minimum gray value  (uint8)\n"
			<< "Format: -set_retro_param lidar_ip patameter_id value\n"
			<< "Format: -set_retro_param -30sp lidar_ip patameter_id value\n"
			<< "Demo:   -set_retro_param 192.168.10.108 4 3\n"
			<< "Demo:   -set_retro_param -30sp 192.168.10.108 4 3\n\n"

			<< "Sample 24 : set Adhesion parameters\n"
			<< "1. angle hor min:  Minimum horizontal angle range (int32)\n"
			<< "2. angle hor max:  Maximum horizontal angle range (int32)\n"
			<< "3. angle ver min:  Minimum vertical angle range (int32)\n"
			<< "4. angle ver max:  Maximum vertical angle range (int32)\n"
			<< "5. angle hor res:  Horizontal angle resolution (float)\n"
			<< "6. angle ver res:  Vertical angle resolution (float)\n"
			<< "7. diff thres:     Delete point threshold (float)\n"
			<< "8. dis limit:      Maximum processing distance  (float)\n"
			<< "9. min diff:       Distance difference between nearest and farthest points (float)\n"
			<< "Format: -set_adhesion_param lidar_ip patameter_id value\n"
			<< "Format: -set_adhesion_param -30sp lidar_ip patameter_id value\n"
			<< "Demo:   -set_adhesion_param 192.168.10.108 3 -40\n"
			<< "Demo:   -set_adhesion_param -30sp 192.168.10.108 3 -40\n\n"

			<< "Sample 25 : get algorithm parameters\n"
			<< "Format: -get_algo_param lidar_ip\n"
			<< "Format: -get_algo_param -30sp lidar_ip\n"
			<< "Demo:   -get_algo_param 192.168.10.108\n"
			<< "Demo:   -get_algo_param -30sp 192.168.10.108\n\n"

			<< "Sample 26 : config delete close points mode\n"
			<< "Format: -config_delete_points lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_delete_points_multi lidar1_ip lidar2_ip(MaxLidarCount:4) mode\n"
			<< "Format: -config_delete_points -30sp lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_delete_points_multi -30sp lidar1_ip lidar2_ip(MaxLidarCount:4) mode\n"
			<< "Demo:   -config_delete_points 192.168.10.108 0\n"
			<< "Demo:   -config_delete_points_multi 192.168.10.108 192.168.10.109 0\n"
			<< "Demo:   -config_delete_points -30sp 192.168.10.108 0\n"
			<< "Demo:   -config_delete_points_multi -30sp 192.168.10.108 192.168.10.109 0\n\n"

			<< "Sample 27 : config adhesion mode\n"
			<< "Format: -config_adhesion lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_adhesion_multi lidar1_ip lidar2_ip(MaxLidarCount:4) mode\n"
			<< "Format: -config_adhesion -30sp lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_adhesion_multi -30sp lidar1_ip lidar2_ip(MaxLidarCount:4) mode\n"
			<< "Demo:   -config_adhesion 192.168.10.108 0\n"
			<< "Demo:   -config_adhesion_multi 192.168.10.108 192.168.10.109 0\n"
			<< "Demo:   -config_adhesion -30sp 192.168.10.108 0\n"
			<< "Demo:   -config_adhesion_multi -30sp 192.168.10.108 192.168.10.109 0\n\n"

			<< "Sample 28 : set calibration data from file\n"
			<< "Format: -set_cal lidar_ip savefilename\n"
			<< "Format: -set_cal -30sp lidar_ip savefilename\n"
			<< "Demo:   -set_cal 192.168.10.108 device.cal\n"
			<< "Demo:   -set_cal -30sp 192.168.10.108 device.cal\n\n"

			<< "Sample 29 : get channel data to file\n"
			<< "Format: -get_channel lidar_ip savefilename mode(0 for origin, 1 for currently in use)\n"
			<< "Demo:   -get_channel 192.168.10.108 channel.coe 0\n\n"

			<< "Sample 30 : get channel list data to file\n"
			<< "Format: -get_channel_list lidar_ip savefilename\n"
			<< "Demo:   -get_channel_list 192.168.10.108 channel_list.coe\n\n"

			<< "Sample 31 : config echo mode\n"
			<< "1. Single first return\n"
			<< "2. Singe strongest return\n"
			<< "3. Singe last return\n"
			<< "4. Double(first and strongest return)\n"
			<< "5. Double(first and last return)\n"
			<< "6. Double(strongest and last return)\n"
			<< "Format: -set_echo lidar_ip mode(1 - 6)\n"
			<< "Format: -set_echo -30sp lidar_ip mode\n"
			<< "Demo:   -set_echo 192.168.10.108 1\n"
			<< "Demo:   -set_echo -30sp 192.168.10.108 1\n\n"

			<< "Sample 32 : get train log\n"
			<< "Format: -get_train_log lidar_ip savefilename\n"
			<< "Demo:   -get_train_log 192.168.10.108 train_log.txt\n\n"

			<< "Sample 33 : get lidar log file\n"
			<< "Format: -get_log lidar_ip savefilename\n"
			<< "Format: -get_log -30sp lidar_ip savefilename\n"
			<< "Demo:   -get_log 192.168.10.108 log.txt\n"
			<< "Demo:   -get_log -30sp 192.168.10.108 log.txt\n\n"

			<< "Sample 34 : set lidar cover file\n"
			<< "Format: -set_cover_file lidar_ip filename mode(0:for ML30B1/ML30SB1 , 1:for ML30SB2)\n"
			<< "Demo:   -set_cover_file 192.168.10.108 cover.txt 0\n"
			<< "Demo:   -set_cover_file 192.168.10.108 cover.txt 1\n\n"

			<< "Sample 35 : set lidar adc_algorithm parameter file\n"
			<< "Format: -set_adc_algo_param_file lidar_ip input_dir mode output_dir(Optional)\n"
			<< "    input_dir: The input_dir should contain a folder that the name is the lidar`s serial number.\n"
			<< "         mode: 0 - First read the parameter configuration file from input_dir, then get the adc_algorithm_parameter flash \n                   data from the lidar, update the adc_algorithm_parameter flash data through the parameter configuration file,\n                   finally download the updated adc_algorithm_parameter flash data to the lidar\n"
			<< "               1 - Download adc_algorithm_parameter file to lidar directly from input_dir.\n"
			<< "   output_dir: If mode is set to 0 and output_dir is not empty, save new adc_algorithm_parameter file in output_dir.\n"
			<< "Format: -set_adc_algo_param_file_multi lidar1_ip lidar2_ip(MaxLidarCount:4) input_dir mode output_dir(Optional)\n"
			<< "Demo:   -set_adc_algo_param_file 192.168.10.108 /path-to-sn-folder 0\n"
			<< "Demo:   -set_adc_algo_param_file 192.168.10.108 /path-to-sn-folder 0 /path-to-save-folder\n"
			<< "Demo:   -set_adc_algo_param_file 192.168.10.108 /path-to-sn-folder 1\n"
			<< "Demo:   -set_adc_algo_param_file_multi 192.168.10.108 192.168.10.109 /path-to-sn-folder 0\n"
			<< "Demo:   -set_adc_algo_param_file_multi 192.168.10.108 192.168.10.109 /path-to-sn-folder 0 /path-to-save-folder\n"
			<< "Demo:   -set_adc_algo_param_file_multi 192.168.10.108 192.168.10.109 /path-to-sn-folder 1\n\n"

			<< "Sample 36 : get lidar adc_algorithm parameter file\n"
			<< "Format:  -get_adc_algo_param_file lidar_ip savefilename\n"
			<< "Demo:    -get_adc_algo_param_file 192.168.10.108 adc_algo_param.adc_all\n\n"

			<< "Sample 37 : config layer detection enabale\n"
			<< "Format: -config_layer_detect_enable lidar_ip enable(0 for disable, 1 for enable) \n"
			<< "Demo:   -config_layer_detect_enable 192.168.10.108 0\n\n"

			<< "Sample 38 : config PL delete near point level\n"
			<< "Format: -config_pl_dnpl lidar_ip mode(0 for forced deletion, 1 for weak deletion) \n"
			<< "Demo:   -config_pl_dnpl 192.168.10.108 0\n\n"

			<< "############################# END  GUIDE ################################\n\n"
            ;
        getchar();
        return 0;
    }

	init_log(argc, argv);

	zvision::set_ml30splus_b1_ep_mode_enable(false);

	std::string opt = std::string(argv[1]);
	zvision::DeviceType tp = zvision::DeviceType::LidarUnknown;
	std::string appname = "";
	int argc_offset = 0;
	std::string lidar_ip = "";

	using Param = std::map<std::string, std::string>;
	std::map<std::string, std::string> paras;
	ParamResolver::GetParameters(argc, argv, paras, appname);
	if (paras.find("-30sp") != paras.end()) {
		tp = zvision::DeviceType::LidarMl30SA1Plus;
		argc_offset = 1;
		lidar_ip = std::string(argv[3]);
	}
	else {
		lidar_ip = std::string(argv[2]);
	}

	// Get lidar list
	std::vector<std::string> lidars_ip;
	{
		std::set<std::string> temp;
		for (int i = 0; i < argc; i++) {
			std::string ip(argv[i]);
			// filter ip
			if (AssembleIpString(ip)) {
				if (temp.insert(ip).second)
					lidars_ip.push_back(ip);
			}
		}
		// check
		if (lidars_ip.size() > 4)
			lidars_ip = std::vector<std::string>{ std::begin(lidars_ip),std::begin(lidars_ip) + 4 };
	}

	if (0 == std::string(argv[1]).compare("-config_mac") && argc == (4 + argc_offset))
		//Sample code 0 : Set lidar's mac address
		sample_config_lidar_mac_address(lidar_ip, std::string(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-config_static_ip") && argc == (4 + argc_offset))
		//Sample code 1 : Set lidar's static ip address
		sample_config_lidar_ip(lidar_ip, std::string(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-config_subnet_mask") && argc == (4 + argc_offset))
		//Sample code 2 : Set lidar's subnet mask
		sample_config_lidar_subnet_mask(lidar_ip, std::string(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-config_dst_ip") && argc == (4 + argc_offset))
		//Sample code 3 : Set lidar's udp destination ip address
		sample_config_lidar_udp_destination_ip(lidar_ip, std::string(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-config_dst_port") && argc == (4 + argc_offset))
		//Sample code 4 : Set lidar's udp destination port
		sample_config_lidar_udp_destination_port(lidar_ip, std::atoi(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-config_retro") && argc == (4 + argc_offset))
		//Sample code 5 : Set lidar's retro function
		sample_config_lidar_retro_enable(lidar_ip, std::atoi(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-config_time_sync") && argc == (4 + argc_offset))
		//Sample code 6 : Set lidar's time sync mode
		sample_config_lidar_time_sync(lidar_ip, zvision::TimestampType(std::atoi(argv[3 + argc_offset])));

	else if (0 == std::string(argv[1]).compare("-query_version") && argc == (3 + argc_offset))
		//Sample code 7 : Query lidar's firmware version
		sample_query_lidar_firmware_version(lidar_ip, tp);

	else if (0 == std::string(argv[1]).compare("-query_sn") && argc == (3 + argc_offset))
		//Sample code 8 : Query lidar's serial number
		 sample_query_lidar_serial_number(lidar_ip, tp);

	else if (0 == std::string(argv[1]).compare("-query_temp") && argc == (3 + argc_offset))
		//Sample code 9 : Query lidar's hardware temperature
		sample_query_lidar_hardware_temperature(lidar_ip);

	else if (0 == std::string(argv[1]).compare("-query_cfg") && argc == (3 + argc_offset))
		//Sample code 10 : Query lidar's configurature
		sample_query_lidar_configuration(lidar_ip, tp);

	else if (0 == std::string(argv[1]).compare("-get_cal") && argc == (4 + argc_offset))
		//Sample code 11 : Get lidar's calibration file by tcp connection.
		sample_get_lidar_calibration(lidar_ip, std::string(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-firmware_update") && argc == (4 + argc_offset))
		//Sample code 12 : Firmware update.
		sample_firmware_update(lidar_ip, std::string(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-reboot") && argc == (3 + argc_offset))
		//Sample code 13 : Reboot lidar by tcp connection.
		sample_reboot_lidar(lidar_ip, tp);

	else if (0 == std::string(argv[1]).compare("-scan_device") && argc == (3 + argc_offset))
		//Sample code 14 : Scan lidar on the heart beat port
		//Notice, this function is supported by the lidar's new firmware kernel version, at least 0.1.20
		sample_scan_lidar_on_heat_beat_port(std::atoi(argv[2 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-retro_p1") && argc == (4 + argc_offset))
		//Sample code 15 : Config lidar retro parameter 1(min ref, [0,100])
		sample_config_lidar_retro_param_min_ref(lidar_ip, std::atoi(argv[3 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-retro_p2") && argc == (4 + argc_offset))
		//Sample code 16 : Config lidar retro parameter 2(point percentage, [0,100])
		sample_config_lidar_retro_param_point_percentage(lidar_ip, std::atoi(argv[3 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-phase_offset_enable") && argc == (4 + argc_offset))
		//Sample code 17 : Config lidar phaseoffset enable
		sample_config_lidar_phase_offset_enable(lidar_ip, std::atoi(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-phase_offset_value") && argc == (4 + argc_offset))
		//Sample code 18 : Config lidar phaseoffset value
		sample_config_lidar_phase_offset_value(lidar_ip, std::atoi(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-set_ptp_cfg") && argc == (4 + argc_offset))
		//Sample code 19 : Config lidar ptp configuration file
		sample_config_lidar_ptp_configuration_file(lidar_ip, std::string(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-get_ptp_cfg") && argc == (4 + argc_offset))
		//Sample code 20 : Get lidar ptp configuration file
		sample_get_lidar_ptp_configuration_to_file(lidar_ip, std::string(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-cali_file_broadcast_enable") && argc == (4 + argc_offset))
		//Sample code 21 : onfig lidar calibration file broadcast enable
		sample_config_lidar_cali_file_broadcast_mode(lidar_ip, std::atoi(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-downsample_mode") && argc == (4 + argc_offset))
		//Sample code 22 : Config lidar downsample mode
		sample_config_lidar_downsample_mode(lidar_ip, std::string(argv[3 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-set_retro_param") && argc == (5 + argc_offset))
		//Sample code 23 : Set lidar retro parameter
		sample_set_lidar_retro_parameters(lidar_ip, std::atoi(argv[3 + argc_offset]), std::string(argv[4 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-set_adhesion_param") && argc == (5 + argc_offset))
		//Sample code 24 : Set lidar adhesion parameter
		sample_set_lidar_adhesion_parameters(lidar_ip, std::atoi(argv[3 + argc_offset]), std::string(argv[4 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-get_algo_param") && argc == (3 + argc_offset))
		//Sample code 25 : Get lidar algorithm parameter(retro and adhesion)
		sample_get_lidar_algorithm_parameters(lidar_ip, tp);

	else if (0 == std::string(argv[1]).compare("-config_delete_points") && argc == (4 + argc_offset))
		//Sample code 26 : Config lidar delete close points enable
		sample_config_lidar_delete_points(lidar_ip, std::atoi(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-config_adhesion") && argc == (4 + argc_offset))
		//Sample code 27 : Config lidar adhesion mode
		sample_config_lidar_adhesion(lidar_ip, std::atoi(argv[3 + argc_offset]), tp);

	// Handling multiple lidars
	else if (0 == std::string(argv[1]).compare("-config_retro_multi"))
		//Sample code 28 : Set lidars` retro function
		sample_config_lidar_retro_multi(lidars_ip, std::atoi(argv[argc-1]), tp);

	else if (0 == std::string(argv[1]).compare("-query_cfg_multi"))
		//Sample code 29 : Query lidars` configurature
		sample_query_lidar_configuration_multi(lidars_ip, tp);

	else if (0 == std::string(argv[1]).compare("-firmware_update_multi"))
		//Sample code 30 : Firmwares update
		sample_firmware_update_multi(lidars_ip, std::string(argv[argc-1]), tp);

	else if (0 == std::string(argv[1]).compare("-reboot_multi"))
		//Sample code 31 : Reboot lidars by tcp connection.
		sample_reboot_lidar_multi(lidars_ip, tp);

	else if (0 == std::string(argv[1]).compare("-phase_offset_enable_multi"))
		//Sample code 32 : Config lidars` phaseoffset enable.
		sample_config_lidar_phase_offset_multi(lidars_ip, std::atoi(argv[argc-1]), tp);

	else if (0 == std::string(argv[1]).compare("-config_delete_points_multi"))
		//Sample code 33 : Config lidars` delete close points enable.
		sample_config_lidar_delete_points_multi(lidars_ip, std::atoi(argv[argc-1]), tp);

	else if (0 == std::string(argv[1]).compare("-config_adhesion_multi"))
		//Sample code 34 : Config lidars` adhesion mode.
		sample_config_lidar_adhesion_multi(lidars_ip, std::atoi(argv[argc-1]), tp);


	else if(0 == std::string(argv[1]).compare("-set_cal") && argc == (4 + argc_offset))
		//Sample code 35 : Set lidar's calibration file by tcp connection.
		sample_config_lidar_calibration(lidar_ip, std::string(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-get_channel") && argc == (5 + argc_offset))
		//Sample code 36 : Get lidar's channel file.
		sample_get_lidar_channel_file(lidar_ip, std::string(argv[3 + argc_offset]), std::atoi(argv[4 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-get_channel_list") && argc == (4 + argc_offset))
		//Sample code 37 : Get lidar's channel list file.
		sample_get_lidar_channel_list_file(lidar_ip, std::string(argv[3 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-set_echo") && argc == (4 + argc_offset))
		//Sample code 38 : Set lidar's echo mode.
		sample_config_lidar_echo_mode(lidar_ip, std::atoi(argv[3 + argc_offset]), tp);

	else if (0 == std::string(argv[1]).compare("-get_train_log") && argc == (4 + argc_offset))
		//Sample code 39 : Get lidar's train log.
		sample_get_lidar_train_log(lidar_ip, std::string(argv[3 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-get_log") && argc == (4 + argc_offset))
		//Sample code 40 : Get lidar's log.
		sample_get_lidar_log(lidar_ip, std::string(argv[3 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-set_cover_file") && argc == (5 + argc_offset))
		//Sample code 41 : Set lidar's cover file.
		sample_set_lidar_cover_file(lidar_ip, std::string(argv[3 + argc_offset]), std::atoi(argv[4 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-set_adc_algo_param_file") && (argc == (5 + argc_offset) || argc == (6 + argc_offset)))
	{
		//Sample code 42: Set lidar's adc_algo_param file.
		if (argc == (5 + argc_offset))
			sample_adc_algo_param_update(lidar_ip, std::string(argv[3 + argc_offset]), std::atoi(argv[4 + argc_offset]));
		else
			sample_adc_algo_param_update(lidar_ip, std::string(argv[3 + argc_offset]), std::atoi(argv[4 + argc_offset]), std::string(argv[5 + argc_offset]));
	}

	else if (0 == std::string(argv[1]).compare("-get_adc_algo_param_file") && argc == (4 + argc_offset))
		//Sample code 43: Set lidar's adc_algo_param file.
		sample_get_adc_algo_param(lidar_ip, std::string(argv[3 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-set_adc_algo_param_file_multi") && (argc == (lidars_ip.size() + 4 + argc_offset) || argc == (lidars_ip.size() + 5 + argc_offset)))
		//Sample code 44: Set multi lidar adc_algo_param file.
		if (argc == (lidars_ip.size() + 4 + argc_offset))
			sample_adc_algo_param_update_multi(lidars_ip, std::string(argv[lidars_ip.size() + 2 + argc_offset]) \
				, std::atoi(argv[lidars_ip.size() + 3 + argc_offset]));
		else
			sample_adc_algo_param_update_multi(lidars_ip, std::string(argv[lidars_ip.size() + 2 + argc_offset]) \
				, std::atoi(argv[lidars_ip.size() + 3 + argc_offset]) \
				, std::string(argv[lidars_ip.size() + 4 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-config_layer_detect_enable") && argc == (4 + argc_offset))
		//Sample code 45 : Config lidar layer detection enable
		sample_config_lidar_layer_detection(lidar_ip, std::atoi(argv[3 + argc_offset]));

	else if (0 == std::string(argv[1]).compare("-config_pl_dnpl") && argc == (4 + argc_offset))
		//Sample code 46 : Config lidar PL delete near point level
		sample_config_lidar_pl_delete_near_point_level(lidar_ip, std::atoi(argv[3 + argc_offset]));

    else
    {
        LOG_F(ERROR, "Invalid parameters.");
        return zvision::InvalidParameter;
    }

    return 0;
}

#if 0// test code
int main(int argc, char** argv)
{
    std::string lidar_ip = "192.168.10.108";

    //Sample code 0 : Set lidar's mac address
    sample_config_lidar_mac_address(lidar_ip, "76-66-66-66-66-66");

    //Sample code 1 : Set lidar's static ip address
    sample_config_lidar_ip(lidar_ip, "192.168.10.108");

    //Sample code 2 : Set lidar's subnet mask
    sample_config_lidar_subnet_mask(lidar_ip, "255.255.255.0");

    //Sample code 3 : Set lidar's udp destination ip address
    sample_config_lidar_udp_destination_ip(lidar_ip, "192.168.10.255");

    //Sample code 4 : Set lidar's udp destination port
    sample_config_lidar_udp_destination_port(lidar_ip, 2368);

    //Sample code 5 : Set lidar's retro function
    sample_config_lidar_retro_enable(lidar_ip, false);

    //Sample code 6 : Set lidar's time sync mode
    sample_config_lidar_time_sync(lidar_ip, zvision::TimestampType::TimestampPtp);

    //Sample code 7 : Query lidar's firmware version
    sample_query_lidar_firmware_version(lidar_ip);

    //Sample code 8 : Query lidar's serial number
    sample_query_lidar_serial_number(lidar_ip);

    //Sample code 9 : Query lidar's hardware temperature
    sample_query_lidar_configuration(lidar_ip);

    //Sample code 10 : Query lidar's configurature
    sample_query_lidar_hardware_temperature(lidar_ip);

    //Sample code 11 : Get lidar's calibration file by tcp connection.
    sample_get_lidar_calibration(lidar_ip);

    //Sample code 12 : Firmware update.
    sample_firmware_update(lidar_ip);

    //Sample code 13 : Reboot lidar by tcp connection.
    sample_reboot_lidar(lidar_ip);

    //Sample code 14 : Scan lidar on the heart beat port
    //Notice, this function is supported by the lidar's new firmware kernel version, at least 0.1.20
    sample_scan_lidar_on_heat_beat_port();

    return 0;
}

#endif
