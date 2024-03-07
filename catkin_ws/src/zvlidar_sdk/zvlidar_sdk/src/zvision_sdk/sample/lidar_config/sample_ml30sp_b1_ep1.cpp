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

//Sample code 1 : Set lidar's mac address
int sample_config_lidar_mac_address(std::string lidar_ip, std::string mac)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DeviceMacAddress(mac);

    if (ret)
        LOG_F(ERROR, "Set device [%s]'s MAC address to [%s] failed, ret = %d.", lidar_ip.c_str(), mac.c_str(), ret);
    else
        LOG_F(INFO, "Set device [%s]'s MAC address to [%s] ok.", lidar_ip.c_str(), mac.c_str());
    return ret;
}

//Sample code 2 : Set lidar's static ip address
int sample_config_lidar_ip(std::string lidar_ip, std::string new_ip)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DeviceStaticIpAddress(new_ip);
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s IP address to [%s] failed, ret = %d.", lidar_ip.c_str(), new_ip.c_str(), ret);
    else
        LOG_F(INFO, "Set device [%s]'s IP address to [%s] ok.", lidar_ip.c_str(), new_ip.c_str());
    return ret;
}

//Sample code 3 : Set lidar's subnet mask
int sample_config_lidar_subnet_mask(std::string lidar_ip, std::string subnetmask)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DeviceSubnetMaskAddress(subnetmask);
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s subnet mask to [%s] failed, ret = %d.", lidar_ip.c_str(), subnetmask.c_str(), ret);
    else
        LOG_F(INFO, "Set device [%s]'s subnet mask to [%s] ok.", lidar_ip.c_str(), subnetmask.c_str());
    return ret;
}

//Sample code 4 : Set lidar's udp destination ip address
int sample_config_lidar_udp_destination_ip(std::string lidar_ip, std::string dst_ip)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DeviceUdpDestinationIpAddress(dst_ip);

    if (ret)
        LOG_F(ERROR, "Set device [%s]'s UDP destination ip to [%s] failed, ret = %d.", lidar_ip.c_str(), dst_ip.c_str(), ret);
    else
        LOG_F(INFO, "Set device [%s]'s UDP destination ip to [%s] ok.", lidar_ip.c_str(), dst_ip.c_str());
    return ret;
}

//Sample code 5 : Set lidar's udp destination port
int sample_config_lidar_udp_destination_port(std::string lidar_ip, int dst_port)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DeviceUdpDestinationPort(dst_port);
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s UDP destination port to [%d] failed, ret = %d.", lidar_ip.c_str(), dst_port, ret);
    else
        LOG_F(INFO, "Set device [%s]'s UDP destination port to [%d] ok.", lidar_ip.c_str(), dst_port);
    return ret;
}

//Sample code 6 : Set lidar's retro function
int sample_config_lidar_retro_enable(std::string lidar_ip, bool enable)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DeviceAlgorithmEnable(zvision::AlgoType::AlgoRetro,enable);

    if (ret)
        LOG_F(ERROR, "Set device [%s]'s retro to [%d] failed, ret = %d.", lidar_ip.c_str(), enable, ret);
    else
        LOG_F(INFO, "Set device [%s]'s retro to [%d] ok.", lidar_ip.c_str(), enable);
    return ret;
}

//Sample code 7 : Set lidar's ptp sync enable
int sample_config_lidar_ptp_sync_enable(std::string lidar_ip, bool enable)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetML30sPlusB1DevicePtpEnable(enable))
        LOG_F(ERROR, "Set device [%s]'s ptp sync enable to [%d] failed, ret = %d.", lidar_ip.c_str(), enable,  ret);
    else
        LOG_F(INFO, "Set device [%s]'s ptp sync enable to [%d] ok.", lidar_ip.c_str(), enable);
    return ret;
}

//Sample code 8 : Query lidar's firmware version
int sample_query_lidar_firmware_version(std::string lidar_ip)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    std::string boot_version, kernel_version;
    zvision::FirmwareVersion version;
	zvision::FirmwareVersion version_bak;
	int	ret = config.GetML30sPlusB1DeviceFirmwareVersion(version, version_bak);

    if (ret)
        LOG_F(ERROR, "Query device [%s]'s firmware version failed, ret = %d.", lidar_ip.c_str(), ret);
    else
    {
        LOG_F(INFO, "Query device [%s]'s firmware version ok.", lidar_ip.c_str());
        LOG_F(INFO, "FPGA version: %u.%u.%u.%u", version.boot_version[0], version.boot_version[1], version.boot_version[2], version.boot_version[3]);
        LOG_F(INFO, "FPGA version bak: %u.%u.%u.%u", version_bak.boot_version[0], version_bak.boot_version[1], version_bak.boot_version[2], version_bak.boot_version[3]);
		LOG_F(INFO, "Embedded version: %u.%u.%u.%u", version.kernel_version[0], version.kernel_version[1], version.kernel_version[2], version.kernel_version[3]);
		LOG_F(INFO, "Embedded version bak: %u.%u.%u.%u", version_bak.kernel_version[0], version_bak.kernel_version[1], version_bak.kernel_version[2], version_bak.kernel_version[3]);
    }
    return ret;
}

//Sample code 9: Query lidar's serial number
int sample_query_lidar_serial_number(std::string lidar_ip)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    std::string serial_number;
	int	ret = config.GetML30sPlusB1DeviceSN(serial_number);
    if (ret)
        LOG_F(ERROR, "Query device [%s]'s serial number failed, ret = %d.", lidar_ip.c_str(), ret);
    else
    {
        LOG_F(INFO, "Query device [%s]'s serial number ok.", lidar_ip.c_str());
        LOG_F(INFO, "Serial number: %s", serial_number.c_str());
    }
    return ret;
}

//Sample code 10 : Query lidar's configurature
int sample_query_lidar_configuration(std::string lidar_ip)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    zvision::DeviceConfigurationInfo info;
	int	ret = config.QueryML30sPlusB1DeviceConfigurationInfo(info);
    if (ret)
        LOG_F(ERROR, "Query device [%s]'s configuration info failed, ret = %d.", lidar_ip.c_str(), ret);
    else
    {
        std::string info_str = zvision::get_ml30splus_b1_cfg_info_string(info);
        LOG_F(INFO, "Query device [%s]'s configuration info ok.", lidar_ip.c_str());
        LOG_F(INFO, "%s", info_str.c_str());
    }
    return ret;
}

//Sample code 11 : Get lidar's calibration file by tcp connection.
int sample_get_lidar_calibration(std::string lidar_ip, std::string savefilename)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.GetML30sPlusB1DeviceCalibrationDataToFile(savefilename))
        LOG_F(ERROR, "Get device [%s]'s calibration data failed, ret = %d.", lidar_ip.c_str(), ret);
    else
    {
        LOG_F(INFO, "Get device [%s]'s calibration data ok.", lidar_ip.c_str());
        LOG_F(INFO, "Calibration data save to file %s.", savefilename.c_str());
    }
    return ret;
}

//Sample code 12 : Firmware update.
int sample_firmware_update(std::string lidar_ip, std::string filename)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.ML30sPlusB1FirmwareUpdate(filename, print_current_progress_multi, false);
    if (ret)
        LOG_F(ERROR, "Update device [%s]'s firmware %s failed, ret = %d.", lidar_ip.c_str(), filename.c_str(), ret);
    else
    {
        LOG_F(INFO, "Update device [%s] fireware %s ok.", lidar_ip.c_str(), filename.c_str());
    }
    return ret;
}

//Sample code 13 : Reboot lidar by tcp connection.
int sample_reboot_lidar(std::string lidar_ip)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.RebootML30sPlusB1Device();
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
    
    if (ret = zvision::LidarTools::ScanML30sPlusB1Device(devices, seconds))
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

//Sample code 15 : Config lidar frame sync enable
int sample_config_lidar_frame_sync_enable(std::string lidar_ip, bool en)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DeviceFrameSyncEnable(en);
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s frame sync enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
    else
        LOG_F(INFO, "Set device [%s]'s frame sync enable to [%d] ok.", lidar_ip.c_str(), en);
    return ret;
}

//Sample code 16 : Config lidar frame syn offset value
int sample_config_lidar_frame_offset_value(std::string lidar_ip, int value_5ns)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DeviceFrameSyncOffset(value_5ns);
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s frame offset value to [%d]x5ns failed, ret = %d.", lidar_ip.c_str(), value_5ns, ret);
    else
        LOG_F(INFO, "Set device [%s]'s frame offset value to [%d]x5ns ok.", lidar_ip.c_str(), value_5ns);
    return ret;
}

//Sample code 17 : Config lidar ptp configuration file
int sample_config_lidar_ptp_configuration_file(std::string lidar_ip, std::string filename)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DevicePtpConfiguration(filename);
    if (ret)
        LOG_F(ERROR, "Set device [%s]'s ptp configuration file to [%s] failed, ret = %d.", lidar_ip.c_str(), filename.c_str(), ret);
    else
        LOG_F(INFO, "Set device [%s]'s ptp configuration file to [%s] ok.", lidar_ip.c_str(), filename.c_str());
    return ret;
}

//Sample code 18 : Get lidar ptp configuration file
int sample_get_lidar_ptp_configuration_to_file(std::string lidar_ip, std::string filename)
{
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.GetML30sPlusB1DevicePtpConfigurationToFile(filename);
    if (ret)
        LOG_F(ERROR, "Get device [%s]'s ptp configuration file to [%s] failed, ret = %d.", lidar_ip.c_str(), filename.c_str(), ret);
    else
        LOG_F(INFO, "Get device [%s]'s ptp configuration file to [%s] ok.", lidar_ip.c_str(), filename.c_str());
    return ret;
}

//Sample code 19 : Config lidar calibration file broadcast enable
int sample_config_lidar_cali_file_broadcast_mode(std::string lidar_ip, bool en)
{
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::CalSendMode mode = zvision::CalSendMode::CalSendDisable;
	int	ret = config.SetML30sPlusB1DeviceCalSendEnable(en);
	if (ret)
		LOG_F(ERROR, "Set device [%s]'s calibration file broadcast enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
	else
		LOG_F(INFO, "Set device [%s]'s calibration file broadcast enable to [%d] ok.", lidar_ip.c_str(), en);
	return ret;
}

//Sample code 20 : Config lidar downsample mode
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

	if (ret = config.SetML30sPlusB1DeviceDownsampleMode(dm))
		LOG_F(ERROR, "Set device [%s]'s downsample mode to [%s] failed, ret = %d.", lidar_ip.c_str(), mode.c_str(), ret);
	else
		LOG_F(INFO, "Set device [%s]'s downsample mode to [%s] ok.", lidar_ip.c_str(), mode.c_str());
	return ret;
}

//Sample code 21 : Config lidar retro parameter
int sample_set_lidar_retro_parameters(std::string lidar_ip,int id, int val) {

	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	switch (id)
	{
	case 1:
		ret = config.SetML30sPlusB1DeviceRetroParam(zvision::RetroParam::RetroMinGray, val);
		break;
	case 2:
		ret = config.SetML30sPlusB1DeviceRetroParam(zvision::RetroParam::RetroMinGrayNum, val);
		break;
	case 3:
		ret = config.SetML30sPlusB1DeviceRetroParam(zvision::RetroParam::RetroDelGrayThres, val);
		break;
	case 4:
		ret = config.SetML30sPlusB1DeviceRetroParam(zvision::RetroParam::RetroDisThres, val);
		break;
	case 5:
		ret = config.SetML30sPlusB1DeviceRetroParam(zvision::RetroParam::RetroLowRangeThres, val);
		break;
	case 6:
		ret = config.SetML30sPlusB1DeviceRetroParam(zvision::RetroParam::RetroHighRangeThres, val);
		break;
	default:
		LOG_F(ERROR, "Invalid parameter id:[%d].", id);
		return -1;
	}
	
	if (ret != 0)
		LOG_F(ERROR, "Set device [%s]'s retro parameters failed, ret = %d.", lidar_ip.c_str(), ret);
	else
		LOG_F(INFO, "Set device [%s]'s retro parameters ok.", lidar_ip.c_str());

	return ret;
}

//Sample code 22 : Config lidar adhesion parameter
int sample_set_lidar_adhesion_parameters(std::string lidar_ip, int id, float val) {
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::DeviceAlgoParam param;
	switch (id)
	{
	case 1:
		ret = config.SetML30sPlusB1DeviceAdhesionParamParam(zvision::AdhesionParam::MinimumHorizontalAngleRange, val);
		break;
	case 2:
		ret = config.SetML30sPlusB1DeviceAdhesionParamParam(zvision::AdhesionParam::MaximumHorizontalAngleRange, val);
		break;
	case 3:
		ret = config.SetML30sPlusB1DeviceAdhesionParamParam(zvision::AdhesionParam::MinimumVerticalAngleRange, val);
		break;
	case 4:
		ret = config.SetML30sPlusB1DeviceAdhesionParamParam(zvision::AdhesionParam::MaximumVerticalAngleRange, val);
		break;
	case 5:
		ret = config.SetML30sPlusB1DeviceAdhesionParamParam(zvision::AdhesionParam::HorizontalAngleResolution, val);
		break;
	case 6:
		ret = config.SetML30sPlusB1DeviceAdhesionParamParam(zvision::AdhesionParam::VerticalAngleResolution, val);
		break;
	case 7:
		ret = config.SetML30sPlusB1DeviceAdhesionParamParam(zvision::AdhesionParam::DeletePointThreshold, val);
		break;
	case 8:
		ret = config.SetML30sPlusB1DeviceAdhesionParamParam(zvision::AdhesionParam::MaximumProcessingRange, val);
		break;
	case 9:
		ret = config.SetML30sPlusB1DeviceAdhesionParamParam(zvision::AdhesionParam::NearFarPointDiff, val);
		break;
	default:
		LOG_F(ERROR, "Invalid parameter id:[%d].", id);
		return -1;
	}

	if (ret != 0)
		LOG_F(ERROR, "Set device [%s]'s adhesion parameters failed, ret = %d.", lidar_ip.c_str(), ret);
	else
		LOG_F(INFO, "Set device [%s]'s adhesion parameters ok.", lidar_ip.c_str());

	return ret;
}

//Sample code 23 : Get lidar algorithm parameter(retro and adhesion)
int sample_get_lidar_algorithm_parameters(std::string lidar_ip) {

	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::DeviceConfigurationInfo info;
	ret = config.GetML30sPlusB1DeviceAlgoParamInUse(info);
	if (ret)
	{
		LOG_F(ERROR, "Get device [%s]'s algorithm parameters failed, ret = %d.", lidar_ip.c_str(), ret);
		return ret;
	}

	{
		LOG_F(INFO, "Get device [%s]'s algorithm parameters ok.", lidar_ip.c_str());
		std::string msg;
		msg += "------  Delete Point  -------\n";
		msg += "switch: " + std::to_string(info.delete_point_enable) + "\n";

		msg += "------  Intensity Smooth  -------\n";
		msg += "switch: " + std::to_string(info.intensity_smooth_enable) + "\n";

		msg += "------  Retro  -------\n";
		msg += "switch: " + std::to_string(info.retro_enable) + "\n";
		msg += "min gray: " + std::to_string(info.algo_param.retro_min_gray) + "\n";
		msg += "min gray num: " + std::to_string(info.algo_param.retro_min_gray_num) + "\n";
		msg += "del range thres: " + std::to_string(info.algo_param.retro_del_gray_thres) + "\n";
		msg += "del gray distance thres: " + std::to_string(info.algo_param.retro_del_gray_dis_thres) + "\n";
		msg += "del near gray thres: " + std::to_string(info.algo_param.retro_near_del_gray_thres) + "\n";
		msg += "del far gray thres: " + std::to_string(info.algo_param.retro_far_del_gray_thres) + "\n";
		
		msg += "-----  Adhesion  -----\n";
		msg += "switch: " + std::to_string(info.adhesion_enable) + "\n";
		msg += "angle hor min: " + std::to_string(info.algo_param.adhesion_angle_hor_min) +"\n";
		msg += "angle hor max: " + std::to_string(info.algo_param.adhesion_angle_hor_max) +"\n";
		msg += "angle ver min: " + std::to_string(info.algo_param.adhesion_angle_ver_min) +"\n";
		msg += "angle ver max: " + std::to_string(info.algo_param.adhesion_angle_ver_max) +"\n";
		msg += "angle hor res: " + std::to_string(info.algo_param.adhesion_angle_hor_res) +"\n";
		msg += "angle ver res: " + std::to_string(info.algo_param.adhesion_angle_ver_res) +"\n";
		msg += "diff thres: " + std::to_string(info.algo_param.adhesion_diff_thres) + "\n";
		msg += "dis limit: " + std::to_string(info.algo_param.adhesion_dis_limit) + "\n";
		msg += "min diff: " + std::to_string(info.algo_param.adhesion_min_diff) + "\n";

		msg += "-----  Dirty Check&Refresh  -----\n";
		msg += "switch check: " + std::to_string(info.dirty_check_enable) + "\n";
		msg += "switch refresh: " + std::to_string(info.dirty_refresh_enable) + "\n";
		msg += "refresh_cycle: " + std::to_string(info.algo_param.dirty_refresh_cycle) + "\n";
		msg += "detect set thre: " + std::to_string(info.algo_param.dirty_detect_set_thre) + "\n";
		msg += "detect reset thre: " + std::to_string(info.algo_param.dirty_detect_reset_thre) + "\n";
		msg += "detect inner thre: " + std::to_string(info.algo_param.dirty_detect_inner_thre) + "\n";
		msg += "detect outer thre: " + std::to_string(info.algo_param.dirty_detect_outer_thre) + "\n";
		LOG_F(INFO, msg.c_str());
	}

	return ret;
}

//Sample code 24 : Config lidar delete close points enable
int sample_config_lidar_delete_points(std::string lidar_ip, bool en)
{
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DeviceAlgorithmEnable(zvision::AlgoType::AlgoDeleteClosePoints, en);
	if (ret)
		LOG_F(ERROR, "Set device [%s]'s delete close points enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
	else
		LOG_F(INFO, "Set device [%s]'s delete close points enable to [%d] ok.", lidar_ip.c_str(), en);
	return ret;
}

//Sample code 25 : Config lidar adhesion enable
int sample_config_lidar_adhesion(std::string lidar_ip, bool en)
{
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	int	ret = config.SetML30sPlusB1DeviceAlgorithmEnable(zvision::AlgoType::AlgoAdhesion, en);
	if (ret)
		LOG_F(ERROR, "Set device [%s]'s adhesion enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
	else
		LOG_F(INFO, "Set device [%s]'s adhesion enable to [%d] ok.", lidar_ip.c_str(), en);
	return ret;
}

/* Handling multiple lidars */
// Muitiple lidars Firmware update.
int sample_firmware_update_multi(std::vector<std::string> lidars_ip, std::string filename)
{
	// start task
	int devs = lidars_ip.size();
	std::vector<std::shared_ptr<std::thread>> thr_vec;
	thr_vec.resize(devs);
	for (int i = 0; i < devs; i++) {
		thr_vec[i].reset(new std::thread(std::bind(&sample_firmware_update, lidars_ip[i], filename)));
	}
	// wait for finish
	for (int i = 0; i < devs; i++) {
		if (thr_vec[i]->joinable())
			thr_vec[i]->join();
	}
	return 0;
}

// Config muitiple lidars adhesion enable.
int sample_config_lidar_adhesion_multi(std::vector<std::string> lidars_ip, bool en) {

	// start task
	for (auto ip : lidars_ip)
		sample_config_lidar_adhesion(ip, en);

	return 0;
}

// Config muitiple lidars delete_points enable.
int sample_config_lidar_delete_points_multi(std::vector<std::string> lidars_ip, bool en) {

	// start task
	for (auto ip : lidars_ip)
		sample_config_lidar_delete_points(ip, en);

	return 0;
}

// Config muitiple lidars retro enable.
int sample_config_lidar_retro_multi(std::vector<std::string> lidars_ip, bool en) {

	// start task
	for (auto ip: lidars_ip)
		sample_config_lidar_retro_enable(ip, en);

	return 0;
}

// Config muitiple lidars frame offset enable.
int sample_config_lidar_frame_sync_enable_multi(std::vector<std::string> lidars_ip, bool en) {

	// start task
	for (auto ip : lidars_ip)
		sample_config_lidar_frame_sync_enable(ip, en);

	return 0;
}

// Config muitiple lidars ptp sync enable.
int sample_config_lidar_ptp_sync_enable_multi(std::vector<std::string> lidars_ip, bool en) {

	// start task
	for (auto ip : lidars_ip)
		sample_config_lidar_ptp_sync_enable(ip, en);

	return 0;
}

// Config muitiple lidars frame offset enable.
int sample_query_lidar_configuration_multi(std::vector<std::string> lidars_ip) {

	// start task
	for (auto ip: lidars_ip)
		sample_query_lidar_configuration(ip);

	return 0;
}

// Reboot lidars by tcp connection.
int sample_reboot_lidar_multi(std::vector<std::string> lidars_ip) {

	// start task
	for (auto ip: lidars_ip)
		sample_reboot_lidar(ip);

	return 0;
}

//Sample code 26 : Set lidar's calibration file by tcp connection.
int sample_config_lidar_calibration(std::string lidar_ip, std::string filename) {
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (ret = config.SetML30sPlusB1DeviceeCalibrationData(filename))
		LOG_F(ERROR, "Set device [%s]`s calibration data failed, ret = %d.", lidar_ip.c_str(), ret);
	else
	{
		LOG_F(INFO, "Set device [%s]`s calibration data ok.", lidar_ip.c_str());
	}
	return ret;
}

// Sample code 27 : Set lidar's echo mode.
int sample_config_lidar_echo_mode(std::string lidar_ip, int id)
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
	int ret = config.SetML30sPlusB1DeviceEchoMode(mode);
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

// Sample code 28 : Get lidar's log file.
int sample_get_lidar_log(std::string lidar_ip, std::string path, bool is_temp = false) {
	
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	std::string context;
	int ret = config.GetML30sPlusB1DeviceLog(context, is_temp);
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

//Sample code 29 : Config lidar dirty check enable
int sample_config_dirty_check_enable(std::string lidar_ip, bool en)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (ret = config.SetML30sPlusB1DeviceDirtyEnable(en))
		LOG_F(ERROR, "Set device [%s]'s dirty check enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
	else
		LOG_F(INFO, "Set device [%s]'s dirty check enable to [%d] ok.", lidar_ip.c_str(), en);
	return ret;
}

//Sample code 30 : Config lidar dirty check enable
int sample_config_intensity_smooth_enable(std::string lidar_ip, bool en)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (ret = config.SetML30sPlusB1DeviceIntensitySmoothEnable(en))
		LOG_F(ERROR, "Set device [%s]'s intensity smooth enable to [%d] failed, ret = %d.", lidar_ip.c_str(), en, ret);
	else
		LOG_F(INFO, "Set device [%s]'s intensity smooth enable to [%d] ok.", lidar_ip.c_str(), en);
	return ret;
}

void init_log(int argc, char* argv[])
{
	loguru::init();
	loguru::add_file("Log/log.txt", loguru::Truncate, loguru::Verbosity_INFO);
	loguru::g_stderr_verbosity = 4;
	loguru::g_preamble_date = 0;
	loguru::g_preamble_time = 0;
	loguru::g_preamble_uptime = 0;
	loguru::g_preamble_thread = 0;
	loguru::g_preamble_file = 0;
	loguru::g_preamble_verbose = 0;
	loguru::g_preamble_pipe = 0;
}

int main(int argc, char** argv)
{
    if (argc <= 2)
    {
        std::cout << "############################# USER GUIDE ################################\n\n"

			<< "Note: Only For ML30S+ B1 EP1(40ms) Device!\n\n"

            << "Sample 1 : config mac address\n"
            << "Format: -config_mac lidar_ip mac_address\n"
            << "Demo:   -config_mac 192.168.10.108 66-66-66-66-66-66\n\n"

            << "Sample 2 : config static ip address\n"
            << "Format: -config_static_ip old_ip new_ip\n"
            << "Demo:   -config_static_ip 192.168.10.108 192.168.10.107\n\n"

            << "Sample 3 : config subnet mask\n"
            << "Format: -config_subnet_mask lidar_ip subnet_mask\n"
            << "Demo:   -config_subnet_mask 192.168.10.108 255.255.255.0\n\n"

            << "Sample 4 : config udp destination ip address\n"
            << "Format: -config_dst_ip lidar_ip dst_ip\n"
			<< "Demo:   -config_dst_ip 192.168.10.108 192.168.10.255\n\n"

            << "Sample 5 : config udp destination port\n"
            << "Format: -config_dst_port lidar_ip port\n"
			<< "Demo:   -config_dst_port 192.168.10.108 2368\n\n"

            << "Sample 6 : config retro mode\n"
            << "Format: -config_retro lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_retro_multi lidar1_ip lidar2_ip (MaxLidarCount:4) mode\n"
            << "Demo:   -config_retro 192.168.10.108 0\n"
			<< "Demo:   -config_retro_multi 192.168.10.108 192.168.10.109 0\n\n"

			<< "Sample 7 : config ptp sync mode\n"
			<< "Format: -config_ptp lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_ptp_multi lidar1_ip lidar2_ip (MaxLidarCount:4) mode\n"
			<< "Demo:   -config_ptp 192.168.10.108 0\n"
			<< "Demo:   -config_ptp_multi 192.168.10.108 192.168.10.109 0\n\n"

            << "Sample 8 : query firmware version\n"
            << "Format: -query_version lidar_ip\n"
			<< "Demo:   -query_version 192.168.10.108\n\n"

            << "Sample 9 : query serial number\n"
            << "Format: -query_sn lidar_ip\n"
			<< "Demo:   -query_sn 192.168.10.108\n\n"

            << "Sample 10 : query configuration\n"
            << "Format: -query_cfg lidar_ip\n"
			<< "Format: -query_cfg_multi lidar1_ip lidar2_ip(MaxLidarCount:4)\n"
            << "Demo:   -query_cfg 192.168.10.108\n"
			<< "Demo:   -query_cfg_multi 192.168.10.108 192.168.10.109\n\n"

            << "Sample 11 : get calibration data to file\n"
            << "Format: -get_cal lidar_ip savefilename\n"
			<< "Demo:   -get_cal 192.168.10.108 device.cal\n\n"

            << "Sample 12 : firmware update\n"
            << "Format: -firmware_update lidar_ip filename\n"
			<< "Format: -firmware_update_multi lidar1_ip lidar2_ip(MaxLidarCount:4) filename\n"
			<< "Demo:   -firmware_update 192.168.10.108 firmware_name.pack\n"
			<< "Demo:   -firmware_update_multi 192.168.10.108 192.168.10.109 firmware_name.pack\n\n"

            << "Sample 13 : reboot\n"
            << "Format: -reboot lidar_ip\n"
			<< "Format: -reboot_multi lidar1_ip lidar2_ip(MaxLidarCount:4) \n"
			<< "Demo:   -reboot 192.168.10.108\n"
			<< "Demo:   -reboot_multi 192.168.10.108 192.168.10.109\n\n"

            << "Sample 14 : scan device\n"
            << "Format: -scan_device scan_time(s)\n"
            << "Demo:   -scan_device 5\n\n"

            << "Sample 15 : frame sync enable(0 for disable, 1 for enable)\n"
            << "Format: -frame_sync_enable lidar_ip mode\n"
			<< "Format: -frame_sync_enable_multi lidar1_ip lidar2_ip(MaxLidarCount:4)\n"
			<< "Demo:   -frame_sync_enable 192.168.10.108 0\n"
			<< "Demo:   -frame_sync_enable_multi 192.168.10.108 192.168.10.109 0\n\n"

            << "Sample 16 : frame offset value\n"
            << "Format: -frame_offset_value lidar_ip value(x5ns)\n"
			<< "Demo:   -frame_offset_value 192.168.10.108 0\n\n"

            << "Sample 17 : config ptp configuration\n"
            << "Format: -set_ptp_cfg lidar_ip filename\n"
			<< "Demo:   -set_ptp_cfg 192.168.10.108 test.txt\n\n"

            << "Sample 18 : get ptp configuration to file\n"
            << "Format: -get_ptp_cfg lidar_ip filename\n"
			<< "Demo:   -get_ptp_cfg 192.168.10.108 test.txt\n\n"

			<< "Sample 19 : calibration file broadcast enabale\n"
			<< "Format: -cali_file_broadcast_enable lidar_ip enable(0 for disable, 1 for enable) \n"
			<< "Demo:   -cali_file_broadcast_enable 192.168.10.108 0\n\n"

			<< "Sample 20 : config downsample mode\n"
			<< "Format: -downsample_mode lidar_ip mode( none: no downsample, 1/2: 50% downsample, 1/4: 25% downsample)\n"
			<< "Demo:   -downsample_mode 192.168.10.108 none\n\n"

			<< "Sample 21 : set Retro parameters\n"
			<< "1. min gray:                  Minimum gray value  (uint8)\n"
			<< "2. min gray num:              Minimum gray value num threshold  (int8)\n"
			<< "3. del gray thres:            Delete point grayscale range threshold  (uint16)\n"
			<< "4. del gray distance thres:   Delete point grayscale to distance threshold  (uint16)\n"
			<< "5. near del gray thres:       Delete near points grayscale threshold  (int8)\n"
			<< "6. far del gray thres:        Delete far points grayscale threshold  (int8)\n"
			
			<< "Format: -set_retro_param lidar_ip patameter_id value\n"
			<< "Demo:   -set_retro_param 192.168.10.108 2 2\n\n"

			<< "Sample 22 : set Adhesion parameters\n"
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
			<< "Demo:   -set_adhesion_param 192.168.10.108 3 -40\n\n"

			<< "Sample 23 : get algorithm parameters\n"
			<< "Format: -get_algo_param lidar_ip\n"
			<< "Demo:   -get_algo_param 192.168.10.108\n\n"

			<< "Sample 24 : config delete close points mode\n"
			<< "Format: -config_delete_points lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_delete_points_multi lidar1_ip lidar2_ip(MaxLidarCount:4) mode\n"
			<< "Demo:   -config_delete_points 192.168.10.108 0\n"
			<< "Demo:   -config_delete_points_multi 192.168.10.108 192.168.10.109 0\n\n"

			<< "Sample 25 : config adhesion mode\n"
			<< "Format: -config_adhesion lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_adhesion_multi lidar1_ip lidar2_ip(MaxLidarCount:4) mode\n"
			<< "Demo:   -config_adhesion 192.168.10.108 0\n"
			<< "Demo:   -config_adhesion_multi 192.168.10.108 192.168.10.109 0\n\n"

			<< "Sample 26 : set calibration data from file\n"
			<< "Format: -set_cal lidar_ip savefilename\n"
			<< "Demo:   -set_cal 192.168.10.108 device.cal\n\n"

			<< "Sample 27 : config echo mode\n"
			<< "1. Single first return\n"
			<< "2. Singe strongest return\n"
			<< "3. Singe last return\n"
			<< "4. Double(first and strongest return)\n"
			<< "5. Double(first and last return)\n"
			<< "6. Double(strongest and last return)\n"
			<< "Format: -set_echo lidar_ip mode(1 - 6)\n"
			<< "Demo:   -set_echo 192.168.10.108 1\n\n"
			
			<< "Sample 28 : get lidar log file\n"
			<< "Format: -get_log lidar_ip savefilename\n"
			<< "Demo:   -get_log 192.168.10.108 log.txt\n\n"

			<< "Sample 29 : config dirty check mode\n"
			<< "Format: -config_dirty_check lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Demo:   -config_dirty_check 192.168.10.108 0\n"

			<< "Sample 30 : config intensity smooth mode\n"
			<< "Format: -config_intensity_smooth lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Demo:   -config_intensity_smooth 192.168.10.108 0\n"

			<< "############################# END  GUIDE ################################\n\n"
            ;
        getchar();
        return 0;
    }

	init_log(argc, argv);

	zvision::set_ml30splus_b1_ep_mode_enable(true);

	std::string opt = std::string(argv[1]);
	std::string appname = "";
	std::string lidar_ip = "";

	using Param = std::map<std::string, std::string>;
	std::map<std::string, std::string> paras;
	ParamResolver::GetParameters(argc, argv, paras, appname);
	lidar_ip = std::string(argv[2]);


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

	if (0 == std::string(argv[1]).compare("-config_mac") && argc == 4)
		//Sample code 1 : Set lidar's mac address
		sample_config_lidar_mac_address(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-config_static_ip") && argc == 4)
		//Sample code 2 : Set lidar's static ip address
		sample_config_lidar_ip(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-config_subnet_mask") && argc == 4)
		//Sample code 3 : Set lidar's subnet mask
		sample_config_lidar_subnet_mask(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-config_dst_ip") && argc == 4)
		//Sample code 4 : Set lidar's udp destination ip address
		sample_config_lidar_udp_destination_ip(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-config_dst_port") && argc == 4)
		//Sample code 5 : Set lidar's udp destination port
		sample_config_lidar_udp_destination_port(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-config_retro") && argc == 4)
		//Sample code 6 : Set lidar's retro function
		sample_config_lidar_retro_enable(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-config_ptp") && argc == 4)
		//Sample code 7 : Set lidar's ptp sync mode
		sample_config_lidar_ptp_sync_enable(lidar_ip, zvision::TimestampType(std::atoi(argv[3])));

	else if (0 == std::string(argv[1]).compare("-query_version") && argc == 3)
		//Sample code 8 : Query lidar's firmware version
		sample_query_lidar_firmware_version(lidar_ip);

	else if (0 == std::string(argv[1]).compare("-query_sn") && argc == 3)
		//Sample code 9 : Query lidar's serial number
		sample_query_lidar_serial_number(lidar_ip);

	else if (0 == std::string(argv[1]).compare("-query_cfg") && argc == 3)
		//Sample code 10 : Query lidar's configurature
		sample_query_lidar_configuration(lidar_ip);

	else if (0 == std::string(argv[1]).compare("-get_cal") && argc == 4)
		//Sample code 11 : Get lidar's calibration file by tcp connection.
		sample_get_lidar_calibration(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-firmware_update") && argc == 4)
		//Sample code 12 : Firmware update.
		sample_firmware_update(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-reboot") && argc == 3)
		//Sample code 13 : Reboot lidar by tcp connection.
		sample_reboot_lidar(lidar_ip);

	else if (0 == std::string(argv[1]).compare("-scan_device") && argc == 3)
		//Sample code 14 : Scan lidar on the heart beat port
		//Notice, this function is supported by the lidar's new firmware kernel version, at least 0.1.20
		sample_scan_lidar_on_heat_beat_port(std::atoi(argv[2]));

	else if (0 == std::string(argv[1]).compare("-frame_sync_enable") && argc == 4)
		//Sample code 15 : Config lidar frame offset enable
		sample_config_lidar_frame_sync_enable(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-frame_offset_value") && argc == 4)
		//Sample code 16 : Config lidar frame offset value
		sample_config_lidar_frame_offset_value(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-set_ptp_cfg") && argc == 4)
		//Sample code 17 : Config lidar ptp configuration file
		sample_config_lidar_ptp_configuration_file(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-get_ptp_cfg") && argc == 4)
		//Sample code 18 : Get lidar ptp configuration file
		sample_get_lidar_ptp_configuration_to_file(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-cali_file_broadcast_enable") && argc == 4)
		//Sample code 19 : Config lidar calibration file broadcast enable
		sample_config_lidar_cali_file_broadcast_mode(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-downsample_mode") && argc == 4)
		//Sample code 20 : Config lidar downsample mode
		sample_config_lidar_downsample_mode(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-set_retro_param") && argc == 5)
		//Sample code 21 : Set lidar retro parameter
		sample_set_lidar_retro_parameters(lidar_ip, std::atoi(argv[3]), std::atoi(argv[4]));

	else if (0 == std::string(argv[1]).compare("-set_adhesion_param") && argc == 5)
		//Sample code 22 : Set lidar adhesion parameter
		sample_set_lidar_adhesion_parameters(lidar_ip, std::atoi(argv[3]), std::atof(argv[4]));

	else if (0 == std::string(argv[1]).compare("-get_algo_param") && argc == 3)
		//Sample code 23 : Get lidar algorithm parameter(retro and adhesion)
		sample_get_lidar_algorithm_parameters(lidar_ip);

	else if (0 == std::string(argv[1]).compare("-config_delete_points") && argc == 4)
		//Sample code 24 : Config lidar delete close points enable
		sample_config_lidar_delete_points(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-config_adhesion") && argc == 4)
		//Sample code 25 : Config lidar adhesion mode
		sample_config_lidar_adhesion(lidar_ip, std::atoi(argv[3]));

	// Handling multiple lidars
	else if (0 == std::string(argv[1]).compare("-config_ptp_multi"))
		// Set lidars` retro function
		sample_config_lidar_ptp_sync_enable_multi(lidars_ip, std::atoi(argv[argc - 1]));

	else if (0 == std::string(argv[1]).compare("-config_retro_multi"))
		// Set lidars` retro function
		sample_config_lidar_retro_multi(lidars_ip, std::atoi(argv[argc-1]));

	else if (0 == std::string(argv[1]).compare("-query_cfg_multi"))
		// Query lidars` configurature
		sample_query_lidar_configuration_multi(lidars_ip);

	else if (0 == std::string(argv[1]).compare("-firmware_update_multi"))
		// Firmwares update
		sample_firmware_update_multi(lidars_ip, std::string(argv[argc-1]));

	else if (0 == std::string(argv[1]).compare("-reboot_multi"))
		// Reboot lidars by tcp connection.
		sample_reboot_lidar_multi(lidars_ip);

	else if (0 == std::string(argv[1]).compare("-frame_sync_enable_multi"))
		// Config lidars` phase offset enable.
		sample_config_lidar_frame_sync_enable_multi(lidars_ip, std::atoi(argv[argc - 1]));

	else if (0 == std::string(argv[1]).compare("-config_delete_points_multi"))
		// Config lidars` delete close points enable.
		sample_config_lidar_delete_points_multi(lidars_ip, std::atoi(argv[argc - 1]));

	else if (0 == std::string(argv[1]).compare("-config_adhesion_multi"))
		// Config lidars` adhesion mode.
		sample_config_lidar_adhesion_multi(lidars_ip, std::atoi(argv[argc - 1]));

	else if (0 == std::string(argv[1]).compare("-set_cal") && argc == 4)
		//Sample code 26 : Set lidar's calibration file by tcp connection.
		sample_config_lidar_calibration(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-set_echo") && argc == 4)
		//Sample code 27 : Set lidar's echo mode.
		sample_config_lidar_echo_mode(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-get_log") && argc == 4)
		//Sample code 28 : Get lidar's log.
		sample_get_lidar_log(lidar_ip, std::string(argv[3]));
	
	else if (0 == std::string(argv[1]).compare("-config_dirty_check") && argc == 4)
		//Sample code 29 : Config lidar dirty check enable.
		sample_config_dirty_check_enable(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-config_intensity_smooth") && argc == 4)
		//Sample code 30 : Config lidar intensity smooth enable.
		sample_config_intensity_smooth_enable(lidar_ip, std::atoi(argv[3]));


    else
    {
        LOG_F(ERROR, "Invalid parameters.");
        return zvision::InvalidParameter;
    }

    return 0;
}
