/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "tcp_client.h"
#include "tools.h"
#include <set>
#include <math.h>
#include <cstring>
#include <iostream>
#include <functional>
#include <fstream>
#include <string>
#include <sstream>
#include <thread>
#include <ros/ros.h>
#include <arpa/inet.h>
#include "tcp_client.h"

namespace zvision {


    bool LidarTools::CheckDeviceRet(std::string ret)
    {
        if(ret.size() == 4)
        {
            return (0x00 == ret[2]) && (0x00 == ret[3]);
        }
        else if(ret.size() == 7)
        {
            // check flg
            if(ret[4] != 0x00)
                return false;
            // check sum
            uint16_t val = GetCheckSum(ret.substr(0,5));
            uint16_t chk_sum = ret[5] << 8 | ret[6];
            return val == chk_sum;
        }

        return false;
    }

    int LidarTools::GetOnlineML30SPlusB1CalibrationData(std::string ip, CalibrationData& cal)
    {
        cal.model = Unknown;
        TcpClient client(1000, 1000, 1000);
        int ret = client.Connect(ip);
        if (ret)
        {
            ROS_ERROR_STREAM("Connect error: " << client.GetSysErrorCode());
            return -1;
        }

        // set send cmd
        const int send_len = 9;
        char mlxs_cmd[send_len] = { (char)0xBA, (char)0x00, (char)0x01, (char)0x00 , (char)0x02, (char)0x0B, (char)0x02, (char)0x00, (char)0x00 };
        uint16_t chk_sum = GetCheckSum(std::string(mlxs_cmd, 7));
        mlxs_cmd[7] = (chk_sum >> 8) & 0xFF;
        mlxs_cmd[8] = chk_sum & 0xFF;
        std::string cmd(mlxs_cmd, send_len);
        // send cmd
        if (client.SyncSend(cmd, send_len))
        {
            ROS_ERROR_STREAM("Send cali cmd error: " << client.GetSysErrorCode());
            return -1;
        }
        // get ret part 1
        const int ret_len1 = 5;
        std::string recv_ret1(ret_len1, 'x');
        if (client.SyncRecv(recv_ret1, ret_len1))
        {
            ROS_ERROR_STREAM("Receive cali ret1 error: " << client.GetSysErrorCode());
            return -1;
        }
        // get ret part 2 len
        uint16_t ret_len2 = 0;
        NetworkToHostShort((uint8_t*)(recv_ret1.c_str() +2), (char*)(&ret_len2));
        if (ret_len2 != 0x02 && ret_len2 != 0x06)
        {
            int len = client.GetAvailableBytesLen();
            if (len > 0) {
                std::string temp(len, 'x');
                client.SyncRecv(temp, len);
            }
            ROS_ERROR_STREAM("Get cali ret error, data len not valid:" << ret_len2);
            return -1;
        }
        // get ret part 2
        std::string recv_ret2(ret_len2, 'x');
        if (client.SyncRecv(recv_ret2, ret_len2))
        {
            ROS_ERROR_STREAM("Receive cali ret2 error: " << client.GetSysErrorCode());
            return -1;
        }
        // recv ret data
        std::string recv_ret = recv_ret1 + recv_ret2;
        // check ret
        bool check = true;
        {
            int recv_ret_len = recv_ret.size();
            // check flg
            if (recv_ret[4] != 0x00)
                check = false;
            // check sum
            if (check)
            {
                uint16_t val = GetCheckSum(recv_ret.substr(0, recv_ret_len - 2));
                uint16_t chk_sum = uint8_t(recv_ret[recv_ret_len-2]) << 8 | uint8_t(recv_ret[recv_ret_len-1]);
                check = (val == chk_sum);
            }
        }

        if (!check)
        {
            ROS_ERROR_STREAM("Check cali ret error:" << client.GetSysErrorCode());
            return -1;
        }

        // get cali data total len
        int total_bytes_len = 0;
        NetworkToHost((uint8_t*)(recv_ret.c_str() + 5), (char*)(&total_bytes_len));

        std::vector<std::string> pkts;
        const int cali_header_len = 30;
        const int chk_sum_len = 2;
        const int new_cali_pkt_data_len = 1024;
        std::string new_cali_header_str = "CAL30S+A1";
        int received_bytes_len = 0;
        std::string cali_recv_buf_left;
        while (received_bytes_len < total_bytes_len)
        {
            // get cali header
            std::string header_recv(cali_header_len, 'x');
            int ret = client.SyncRecv(header_recv, cali_header_len);
            if (ret)
            {
                ROS_ERROR_STREAM("Receive calibration header error.");
                return -1;
            }

            // get cali data len
            uint16_t data_len = 0;
            NetworkToHostShort((uint8_t*)header_recv.data() + (cali_header_len - chk_sum_len), (char*)(&data_len));
            if (data_len - chk_sum_len < 0 )
            {
                ROS_ERROR_STREAM("Check calibration header error.");
                return -1;
            }

            // get cali data and checksum
            std::string data_recv(data_len, 'x');
            ret = client.SyncRecv(data_recv, data_len);
            if (ret)
            {
                ROS_ERROR_STREAM("Receive calibration data error.");
                return -1;
            }

            // get packet check sum
            uint16_t pkt_chk = 0;
            NetworkToHostShort((uint8_t*)data_recv.data() + (data_len - chk_sum_len), (char*)(&pkt_chk));
            // get packet data
            std::string cali_data = data_recv.substr(0, data_len - chk_sum_len);
            std::string cali_pkt = header_recv + cali_data;
            // check
            uint16_t chk = GetCheckSum(cali_pkt);
            if (chk != pkt_chk)
            {
                ROS_ERROR_STREAM("Check mlxs calibration data error.");
                return -1;
            }

            // check cali data
            {
                unsigned char check_all_00 = 0x00;
                unsigned char check_all_ff = 0xFF;
                for (int i = 0; i < cali_data.size(); i++)
                {
                    check_all_00 |= cali_data[i];
                    check_all_ff &= cali_data[i];
                }
                if (0x00 == check_all_00)
                {
                    ROS_ERROR_STREAM("Check calibration data error, data is all 0x00.");
                    return -1;
                }
                if (0xFF == check_all_ff)
                {
                    ROS_ERROR_STREAM("Check calibration data error, data is all 0xFF.");
                    return -1;
                }
            }

            // get pkt header information
            char header_info[7] = { (char)0x00, (char)0x00, (char)0x00, (char)0x01 , (char)0x00, (char)0x00, (char)0x00};
            header_info[4] = header_recv.at(25);
            // regenerate cali pkts
            std::string data_buf = cali_recv_buf_left + cali_data;
            int pkt_cnt = data_buf.size() / new_cali_pkt_data_len;
            int bytes_left = data_buf.size() % new_cali_pkt_data_len;
            if (pkt_cnt>0)
            {
                for (int i = 0;i< pkt_cnt;i++)
                {
                    uint16_t cur_pkt_cnt = pkts.size();
                    header_info[0] = (cur_pkt_cnt >> 8) & 0xFF;
                    header_info[1] = cur_pkt_cnt & 0xFF;
                    std::string new_data = data_buf.substr(new_cali_pkt_data_len*i, new_cali_pkt_data_len);
                    std::string new_pkt = new_cali_header_str + std::string(header_info, 7) + new_data; // 9 + 7 + 1024
                    pkts.push_back(new_pkt);
                }
            }

            // update bytes buffer left
            cali_recv_buf_left = std::string();
            if (bytes_left > 0)
                cali_recv_buf_left = data_buf.substr(pkt_cnt* new_cali_pkt_data_len, bytes_left);

            // check
            received_bytes_len += (cali_header_len + data_len);
            if (received_bytes_len >= total_bytes_len)
            {
                break;
            }
        }

        // final check
        // we should get 100/200/400 packets
        if (pkts.size() != 100 && pkts.size() != 200 && pkts.size() != 400)
        {
            ROS_ERROR_STREAM("Get calibration data error.");
            return -1;
        }

        //  compute cali data
        for (int i = 0; i < pkts.size(); i++)
        {
            char* cal_data_ = (char*)pkts[i].c_str() + 16;
            int network_data = 0;
            int host_data = 0;
            float* pfloat_data = reinterpret_cast<float *>(&host_data);
            for (int j = 0; j < 128 * 2; j++)
            {
                memcpy(&network_data, cal_data_ + j * 4 , 4); // 4 bytes per data, azimuth, elevation, 16 bytes header
                host_data = ntohl(network_data);
                cal.data.push_back(*pfloat_data);
            }
        }

		{
			std::vector<float> dst;
			dst.resize(cal.data.size(),.0f);
			int id_h = 0;
			int id_l = cal.data.size() / 2;
			for (int i = 0; i < cal.data.size(); i++) {
				if (i / 8 % 2 == 0) {
					dst.at(id_h) = cal.data[i];
					id_h++;
				}
				else {
					dst.at(id_l) = cal.data[i];
					id_l++;
				}
			}
			cal.data = dst;
		}
        cal.model = ML30SPlusB1;
        return 0;
    }

    int LidarTools::GetOnlineMLXSCalibrationData(std::string ip, CalibrationData& cal)
    {
        // set send cmd
        const int send_len = 9;
        char mlxs_cmd[send_len] = { (char)0xBA, (char)0x00, (char)0x00, (char)0x00 , (char)0x02, (char)0x0B, (char)0x02, (char)0x00, (char)0x00};
        uint16_t chk_sum = GetCheckSum(std::string(mlxs_cmd, 8));
        mlxs_cmd[7] = (chk_sum>>8) & 0xFF;
        mlxs_cmd[8] = chk_sum & 0xFF;
        std::string cmd(mlxs_cmd,send_len);

        cal.model = Unknown;
        TcpClient client(1000, 1000, 1000);
        int ret = client.Connect(ip);
        if (ret)
        {
            ROS_ERROR_STREAM("Connect error: " << client.GetSysErrorCode());
            return -1;
        }

        if (client.SyncSend(cmd, send_len))
        {
            ROS_ERROR_STREAM("Send cali cmd error: " << client.GetSysErrorCode());
            client.Close();
            return -1;
        }
        const int recv_len = 7;
        std::string recv(recv_len, 'x');
        if (client.SyncRecv(recv, recv_len))
        {
            ROS_ERROR_STREAM("Receive cali ret error: " << client.GetSysErrorCode());
            client.Close();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            ROS_ERROR_STREAM("Check cali ret error: " << client.GetSysErrorCode());
            client.Close();
            return -1;
        }

        // get mlxs cali data
        uint32_t received_bytes_len = 0;
        uint32_t total_len = (36000 * 3 * 2);
        uint32_t total_bytes_len = total_len * 4;
        cal.model = MLXS;

        const int cali_header_len = 5;
        const int chk_sum_len = 2;
        std::vector<float>& data = cal.data;
        data.clear();
        // cali packet tail
        std::string cali_recv_left;
        while(received_bytes_len < total_bytes_len )
        {
            // receive calibration header
            std::string header_recv(cali_header_len, 'x');
            int ret = client.SyncRecv(header_recv, cali_header_len);
            if (ret)
            {
                ROS_ERROR_STREAM("Receive mlxs calibration header error\n");
                client.Close();
                return -1;
            }
            // chech cali header
            uint8_t* pheader_recv = (uint8_t*)header_recv.data();
            if( (  *(pheader_recv + 0)!=0xBA) \
                || (*(pheader_recv + 1)!=0xAC) \
                || (*(pheader_recv + 4)!=0x00))
            {
                ROS_ERROR_STREAM("Check mlxs calibration header error\n");
                client.Close();
                return -1;
            }

            // get cali data and checksum
            uint16_t data_len = 0;
            NetworkToHostShort((uint8_t*)header_recv.data() + 2,(char*)(&data_len));
            std::string data_recv(data_len, 'x');
            ret = client.SyncRecv(data_recv, data_len);
            if (ret)
            {
                ROS_ERROR_STREAM("Receive mlxs calibration data error\n");
                client.Close();
                return -1;
            }

            // get packet check sum
            uint16_t pkt_chk = 0;
            NetworkToHostShort((uint8_t*)data_recv.data() +data_len - chk_sum_len, (char*)(&pkt_chk));
            // get packet data
            std::string cali_recv = data_recv.substr(0, data_len - chk_sum_len);
            std::string cali_pkt = header_recv + cali_recv;
            // check
            uint16_t chk = GetCheckSum(cali_pkt);
            if(chk != pkt_chk)
            {
                ROS_ERROR_STREAM("Check mlxs calibration data error\n");
                client.Close();
                return -1;
            }

            // check cali data
            {
                unsigned char check_all_00 = 0x00;
                unsigned char check_all_ff = 0xFF;
                for (int i = 0; i < cali_recv.size(); i++)
                {
                    check_all_00 |= cali_recv[i];
                    check_all_ff &= cali_recv[i];
                }
                if (0x00 == check_all_00)
                {
                    ROS_ERROR_STREAM("Check calibration data error, data is all 0x00.\n");
                    client.Close();
                    return -1;
                }
                if (0xFF == check_all_ff)
                {
                    ROS_ERROR_STREAM("Check calibration data error, data is all 0xFF.\n");
                    client.Close();
                    return -1;
                }
            }
            // add data left over by last cali packet
            if(cali_recv_left.size())
            {
                cali_recv = cali_recv_left + cali_recv;
                cali_recv_left = std::string();
            }
            // get cali packet tail
            int fcnt = cali_recv.size() / 4;
            int str_left = cali_recv.size() % 4;
            if(str_left>0)
            {
                cali_recv_left = cali_recv.substr(fcnt*4, str_left);
                cali_recv = cali_recv.substr(0, fcnt*4);
            }

            // parsing cali data
            int network_data = 0;
            int host_data = 0;
            float* pfloat_data = reinterpret_cast<float *>(&host_data);
            unsigned char* pcali = (unsigned char *)cali_recv.c_str();
            for(int i = 0;i<fcnt;i++)
            {
                memcpy(&network_data, pcali + i * 4, 4);
                host_data = ntohl(network_data);
                data.push_back(*pfloat_data);
            }
            // check
            received_bytes_len += cali_recv.size();
            if( received_bytes_len >= total_bytes_len )
            {
                // printf("received_bytes_len: %d\n",received_bytes_len);
               break;
            }

            std::this_thread::sleep_for(std::chrono::microseconds(110));
        }

        // final check
        if(data.size() != total_len)
        {
            ROS_ERROR_STREAM("Get mlxs calibration data error\n");
            return -1;
        }

        return 0;
    }

    int LidarTools::GetOnlineCalibrationData(std::string ip, CalibrationData& cal, LidarType tp)
    {
        if(tp == LidarType::MLXS )
        {
            return GetOnlineMLXSCalibrationData(ip, cal);
        }else if(tp == LidarType::ML30SPlusB1 )
        {
            return GetOnlineML30SPlusB1CalibrationData(ip, cal);
        }

        //std::string ec;
        //const int ppf = 256000; // points per frame, 256000 reserved
        const int ppk = 128; // points per cal udp packet
        //std::unique_ptr<float> angle_data(new float[ppf * 2]); // points( azimuh, elevation);
        //int packet_buffer_size = 1040 * (ppf / ppk) + 4; // 128 points in one packet, buffer reserved for ppf points.
        //std::unique_ptr<unsigned char> packet_data(new unsigned char[packet_buffer_size]);
        const int send_len = 4;
        char cal_cmd[send_len] = { (char)0xBA, (char)0x07, (char)0x00, (char)0x00 };

        // change cmd
        if(tp == LidarType::ML30SPlusA1){
            cal_cmd[1] = (char)0x0D;
            cal_cmd[2] = (char)0x0A;
        }

        std::string cmd(cal_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        cal.model = Unknown;
        TcpClient client(1000, 1000, 1000);
        int ret = client.Connect(ip);
        if (ret)
        {
            ROS_ERROR_STREAM("Connect error: " << client.GetSysErrorCode());
            return -1;
        }


        if (client.SyncSend(cmd, send_len))
        {
            client.Close();
            return -1;
        }

        if (client.SyncRecv(recv, recv_len))
        {
            client.Close();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            client.Close();
            return -1;
        }

        const int cal_pkt_len = 1040;
        std::string cal_recv(cal_pkt_len, 'x');

        //receive first packet to identify the device type
        if (client.SyncRecv(cal_recv, cal_pkt_len))
        {
            client.Close();
            return -1;
        }

        int total_packet = 0;

        std::string dev_code((char *)cal_recv.c_str() + 3, 6);
        uint32_t data_size = 0;
        if (0 == dev_code.compare("30_B1 "))
        {
            total_packet = 235;
            //data_size = (10000 * 3 * 2);
            data_size = ppk * total_packet * 2;// packet must give a full
            cal.model = ML30B1;
        }
        else if (0 == dev_code.compare("30S_A1"))
        {
            total_packet = 400;
            //data_size = (6400 * 8 * 2);
            uint8_t downsample_flg = *((char *)cal_recv.c_str() + 13);
            if(downsample_flg == 1){
                total_packet = 200;
            }else if(downsample_flg == 2){
                 total_packet = 100;
            }
            data_size = (total_packet * 16 * 8 * 2);
            cal.model = ML30SA1;
        }
        else if (0 == dev_code.compare("30S+A1"))
        {
            total_packet = 400;
            //data_size = (6400 * 8 * 2);
            //uint8_t downsample_flg = *((char *)cal_recv.c_str() + 13);
            //if(downsample_flg == 1){
            //   total_packet = 200;
            //}else if(downsample_flg == 2){
            //     total_packet = 100;
            //}
            data_size = (total_packet * 16 * 8 * 2);
            cal.model = ML30SPlusA1;
        }
        else if (0 == dev_code.compare("30S_B1"))
        {
            total_packet = 400;
            data_size = (6400 * 8 * 2);
            cal.model = ML30SA1;
        }
        else if (0 == dev_code.compare("XS    "))
        {
            total_packet = 844;
            data_size = (36000 * 3 * 2);
            cal.model = MLXS;
        }
        else
        {
            ROS_ERROR_STREAM("Calibration packet identify error");
            client.Close();
            return -1;
        }

        //check the data
        unsigned char* check_data = (unsigned char *)cal_recv.c_str();
        unsigned char check_all_00 = 0x00;
        unsigned char check_all_ff = 0xFF;
        for (int i = 0; i < 1040 - 16; i++)
        {
            check_all_00 |= check_data[i];
            check_all_ff &= check_data[i];
        }
        if (0x00 == check_all_00)
        {
            ROS_ERROR_STREAM("Check calibration data error, data is all 0x00.\n");
            client.Close();
            return -1;
        }
        if (0xFF == check_all_ff)
        {
            ROS_ERROR_STREAM("Check calibration data error, data is all 0xFF.\n");
            client.Close();
            return -1;
        }

        std::vector<float>& data = cal.data;
        {
            int network_data = 0;
            int host_data = 0;
            float* pfloat_data = reinterpret_cast<float *>(&host_data);
            for (int i = 0; i < 128 * 2; i++)
            {
                memcpy(&network_data, check_data + i * 4 + 16, 4); // 4 bytes per data, azimuth, elevation, 16 bytes header
                host_data = ntohl(network_data);
                data.push_back(*pfloat_data);
            }
        }


        for (int i = 0; i < total_packet - 1; i++)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(110));
            int ret = client.SyncRecv(cal_recv, cal_pkt_len);
            if (ret)
            {
                ROS_ERROR_STREAM("Receive calibration data error\n");
                client.Close();
                return -1;
            }
            check_data = (unsigned char *)cal_recv.c_str();
            {
                int network_data = 0;
                int host_data = 0;
                float* pfloat_data = reinterpret_cast<float *>(&host_data);
                for (int i = 0; i < 128 * 2; i++)
                {
                    memcpy(&network_data, check_data + i * 4 + 16, 4); // 4 bytes per data, azimuth, elevation, 16 bytes header
                    host_data = ntohl(network_data);
                    data.push_back(*pfloat_data);
                }
            }
        }

        // now we reorder the cali.data   0~15... -> 0~7... + 8~15...
		if (cal.model == ML30SPlusA1) {
			std::vector<float> dst;
			dst.resize(data.size(),.0f);
			int id_h = 0;
			int id_l = data.size() / 2;
			for (int i = 0; i < data.size(); i++) {
				if (i / 8 % 2 == 0) {
					dst.at(id_h) = data[i];
					id_h++;
				}
				else {
					dst.at(id_l) = data[i];
					id_l++;
				}
			}
			data = dst;
		}

        if (client.SyncRecv(recv, recv_len))
        {
            ROS_ERROR_STREAM("Recv ack error.\n");
            client.Close();
            return -1;
        }
        //printf("recv ack ok.\n");
        if (!CheckDeviceRet(recv))
        {
            ROS_ERROR_STREAM("Check ack error.\n");
            client.Close();
            return -1;
        }

        if(data_size != data.size())
        {
            ROS_ERROR("Calbration data size [%6lu] is not valid, [%6u] wanted.\n", data.size(), data_size);
            return -1;
        }

        return 0;
    }

    int LidarTools::ReadCalibrationFile(std::string filename, CalibrationData& cal)
    {
        const int file_min_lines = 4;
        std::ifstream file;
        file.open(filename, std::ios::in);
        std::string line;
        int ret = 0;
        cal.model = Unknown;
        if (file.is_open())
        {
            std::vector<std::vector<std::string>> lines;
            while (std::getline(file, line))
            {
                if (line.size() > 0)
                {
                    std::istringstream iss(line);
                    std::vector<std::string> datas;
                    std::string data;
                    while (iss >> data)
                    {
                        datas.push_back(data);
                    }
                    lines.push_back(datas);
                }
            }
            file.close();

            if (lines.size() < file_min_lines)
            {
                ROS_ERROR("Calbration file line size [%d] is not valid.\n", lines.size());
                return -1;
            }

            // filter #
            int curr = 0;
            for (auto& line : lines)
            {
                if ('#' != line[0][0])
                    break;
                curr++;
            }

            // read version info
            std::string version_str;
            if ((lines[curr].size() >= 2) && (lines[curr][0] == "VERSION"))
            {
                version_str = lines[curr][lines[curr].size() - 1];
                curr++;
            }

            // read scan mode
            std::string mode_str;
            if ((lines[curr].size() >= 2) && (lines[curr][0] == "Mode"))
            {
                mode_str = lines[curr][lines[curr].size() - 1];
                curr++;
            }
            int ret = 0;
            if (mode_str.size())
            {
                if ("MLX_A1_190" == mode_str)
                {
                    if ((lines.size() - curr) < 38000)//check the file lines
                        return -1;

                    cal.data.resize(38000 * 3 * 2);
                    for (int i = 0; i < 38000; i++)
                    {
                        const int column = 7;

                        std::vector<std::string>& datas = lines[i + curr];
                        if (datas.size() != column)
                        {
                            ret = -1;
                            break;
                        }
                        for (int j = 1; j < column; j++)
                        {
                            cal.data[i * 6 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                        }
                    }
                    cal.model = MLXA1;
                }
                else if (("ML30S_160_1_2" == mode_str) || ("ML30S_160_1_4" == mode_str))
                {
                    int group = 3200;
                    if ("ML30S_160_1_4" == mode_str)
                    {
                        group = 1600;
                    }

                    if ((lines.size() - curr) < group)
                        return -1;

                    cal.data.resize(group * 8 * 2);
                    for (int i = 0; i < group; i++)
                    {
                        const int column = 17;
                        std::vector<std::string>& datas = lines[i + curr];
                        if (datas.size() != column)
                        {
                            ret = -1;
                            ROS_ERROR_STREAM("Resolve calibration file data error.");
                            break;
                        }
                        for (int j = 1; j < column; j++)
                        {
                            cal.data[i * 16 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                        }
                    }
                   cal.model = ML30SA1;
                }
                else if("ML30SPlus_160" == mode_str ){
                    int group = 6400;
                    cal.model = ML30SPlusA1;
                    if ((lines.size() - curr) < group)
                        return -1;

                    cal.data.resize(group * 8 * 2);
                    const int column = 17;
                    const int lpos = cal.data.size() / 2;
                    for (int i = 0; i < group; i++)
                    {
                        std::vector<std::string>& datas = lines[i+curr];
                        if (datas.size() != column)
                        {
                            ret = -1;
                            break;
                        }

                        // for line with fov0 fov1 fov2 fov3 fov4 fov5 fov6 fov7
                        for (int j = 1; j < column; j++)
                        {
                            if (j <= column / 2) // for H view
                                cal.data[i * 8 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                            else // for L view
                                cal.data[i * 8 + j - 1 - column / 2 + lpos] = static_cast<float>(std::atof(datas[j].c_str()));
                        }
                    }
                }
                else if ("MLXs_180" == mode_str)
                {
                    if ((lines.size() - curr) < 36000)//check the file lines
                        return -1;

                    cal.data.resize(36000 * 3 * 2);
                    for (int i = 0; i < 36000; i++)
                    {
                        const int column = 7;

                        std::vector<std::string>& datas = lines[i + curr];
                        if (datas.size() != column)
                        {
                            ret = -1;
                            break;
                        }
                        for (int j = 1; j < column; j++)
                        {
                            cal.data[i * 6 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                        }
                    }
                    cal.model = MLXS;
                }
                else
                {
                    return -1;
                }
            }
            else
            {
                if (10000 == lines.size())
                {
                    cal.data.resize(60000);
                    for (int i = 0; i < 10000; i++)
                    {
                        const int column = 7;

                        std::vector<std::string>& datas = lines[i];
                        if (datas.size() != column)
                        {
                            ret = -1;
                            ROS_ERROR_STREAM("Resolve calibration file data error.");
                            break;
                        }
                        for (int j = 1; j < column; j++)
                        {
                            int fov = (j - 1) % 3;
                            if (0 == ((j - 1) / 3))//azimuth
                            {
                                cal.data[i * 6 + fov * 2] = static_cast<float>(std::atof(datas[j].c_str()));
                            }
                            else//elevation
                            {
                                cal.data[i * 6 + fov * 2 + 1] = static_cast<float>(std::atof(datas[j].c_str()));
                            }
                        }
                    }
                    cal.model = ML30B1;
                }
                else if (6400 == lines.size())
                {
                    cal.data.resize(6400 * 8 * 2);
                    for (int i = 0; i < 6400; i++)
                    {
                        const int column = 17;

                        std::vector<std::string>& datas = lines[i];
                        if (datas.size() != column)
                        {
                            ret = -1;
                            ROS_ERROR_STREAM("Resolve calibration file data error.");
                            break;
                        }
                        for (int j = 1; j < column; j++)
                        {
                            cal.data[i * 16 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                        }
                    }
                    cal.model = ML30SA1;
                }
                else if (32000 == lines.size())
                {
                    cal.data.resize(32000 * 3 * 2);
                    for (int i = 0; i < 32000; i++)
                    {
                        const int column = 7;

                        std::vector<std::string>& datas = lines[i];
                        if (datas.size() != column)
                        {
                            ret = -1;
                            ROS_ERROR_STREAM("Resolve calibration file data error.");
                            break;
                        }
                        for (int j = 1; j < column; j++)
                        {
                            cal.data[i * 6 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                        }
                    }
                    cal.model = MLX;
                }
                else if (38000 < lines.size())
                {
                    cal.data.resize(38000 * 3 * 2);
                    int f = 0;
                    for(f = 0; f < lines.size();f++)
                    {
                        std::vector<std::string>& datas = lines[f];
                        if(datas.size() && (datas[0] == std::string("1")))
                            break;
                    }
                    for (int i = 0; i < 38000; i++)
                    {
                        const int column = 7;
                        std::vector<std::string>& datas = lines[i + f];
                        if (datas.size() != column)
                        {
                            ret = -1;
                            ROS_ERROR_STREAM("Resolve calibration file data error.");
                            break;
                        }
                        for (int j = 1; j < column; j++)
                        {
                            cal.data[i * 6 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                        }
                    }
                    cal.model = MLXA1;
                }
                else
                {
                    cal.model = Unknown;
                    ret = -1;
                    ROS_ERROR_STREAM("Invalid calibration file length.");
                }
            }

            return ret;
        }
        else
        {
            ROS_ERROR_STREAM("Open calibration file error.");
            return -1;
        }
    }

    void LidarTools::ComputeCalibrationData(CalibrationData& cal, PointCalibrationTable& cal_lut)
    {
        cal_lut.data.resize(cal.data.size() / 2);
        cal_lut.model = cal.model;
        if (ML30B1 == cal.model)
        {
            for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
            {
                float azi = static_cast<float>(cal.data[i * 2] / 180.0 * 3.1416);
                float ele = static_cast<float>(cal.data[i * 2 + 1] / 180.0 * 3.1416);

                PointCalibrationData& point_cal = cal_lut.data[i];
                point_cal.ele = ele;
                point_cal.azi = azi;
                point_cal.cos_ele = std::cos(ele);
                point_cal.sin_ele = std::sin(ele);
                point_cal.cos_azi = std::cos(azi);
                point_cal.sin_azi = std::sin(azi);
            }
        }
        else if (ML30SA1 == cal.model)
        {
            const int start = 8;
            int fov_index[start] = { 0, 6, 1, 7, 2, 4, 3, 5 };
            for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
            {
                int start_number = i % start;
                int group_number = i / start;
                int point_numer = group_number * start + fov_index[start_number];
                float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                PointCalibrationData& point_cal = cal_lut.data[i];
                point_cal.ele = ele;
                point_cal.azi = azi;
                point_cal.cos_ele = std::cos(ele);
                point_cal.sin_ele = std::sin(ele);
                point_cal.cos_azi = std::cos(azi);
                point_cal.sin_azi = std::sin(azi);
            }
        }
        else if(ML30SPlusA1 == cal.model || ML30SPlusB1 == cal.model){
            const int start = 4;
			int fov_index[start] = { 0, 1, 2, 3 };
			for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
			{
				int start_number = i % start;
				int group_number = i / start;
				int point_numer = group_number * start + fov_index[start_number];
				float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
				float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

				PointCalibrationData& point_cal = cal_lut.data[i];
				point_cal.cos_ele = std::cos(ele);
				point_cal.sin_ele = std::sin(ele);
				point_cal.cos_azi = std::cos(azi);
				point_cal.sin_azi = std::sin(azi);
				point_cal.azi = azi;
				point_cal.ele = ele;
			}

        }
        else if (MLX == cal.model)
        {
            const int start = 3;
            int fov_index[start] = { 2, 1, 0 };
            for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
            {
                int start_number = i % start;
                int group_number = i / start;
                int point_numer = group_number * start + fov_index[start_number];
                float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                PointCalibrationData& point_cal = cal_lut.data[i];
                point_cal.ele = ele;
                point_cal.azi = azi;
                point_cal.cos_ele = std::cos(ele);
                point_cal.sin_ele = std::sin(ele);
                point_cal.cos_azi = std::cos(azi);
                point_cal.sin_azi = std::sin(azi);
            }
        }
        else if ((MLXA1 == cal.model) || (MLXS == cal.model))
        {
            const int start = 3;
            int fov_index[start] = { 0, 1, 2 };
            for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
            {
                int start_number = i % start;
                int group_number = i / start;
                int point_numer = group_number * start + fov_index[start_number];
                float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                PointCalibrationData& point_cal = cal_lut.data[i];
                point_cal.ele = ele;
                point_cal.azi = azi;
                point_cal.cos_ele = std::cos(ele);
                point_cal.sin_ele = std::sin(ele);
                point_cal.cos_azi = std::cos(azi);
                point_cal.sin_azi = std::sin(azi);
            }
        }
        else
        {

        }
    }

    std::string LidarTools::GetDeviceTypeString(LidarType tp)
    {
        std::string dev_string = "Unknown";
        switch (tp)
        {
        case ML30B1:
            dev_string = "ML30B1";
            break;
        case ML30SA1:
            dev_string = "ML30SA1";
            break;
        case ML30SPlusA1:
            dev_string = "ML30S+A1";
            break;
        case ML30SPlusB1:
            dev_string = "ML30S+B1";
            break;
        case MLX:
            dev_string = "MLX";
            break;
        case MLXA1:
            dev_string = "MLXA1";
            break;
        case MLXS:
            dev_string = "MLXS";
            break;
        default:
            break;
        }
        return dev_string;
    }

    LidarType LidarTools::GetDeviceTypeFromTypeString(std::string tp)
    {
        LidarType dev_ty = LidarType::Unknown;
        if(tp == "ML30B1")
            dev_ty = LidarType::ML30B1;
        else if(tp == "ML30SA1")
            dev_ty = LidarType::ML30SA1;
        else if(tp == "ML30S+A1")
            dev_ty = LidarType::ML30SPlusA1;
        else if(tp == "ML30S+B1")
            dev_ty = LidarType::ML30SPlusB1;
        else if(tp == "MLX")
            dev_ty = LidarType::MLX;
        else if(tp == "MLXA1")
            dev_ty = LidarType::MLXA1;
        else if(tp == "MLXS")
            dev_ty = LidarType::MLXS;
        else
            dev_ty = LidarType::Unknown;

        return dev_ty;
    }

    void LidarTools::ComputePointLineNumber(PointCalibrationTable& cal_lut, std::vector<int>& line_numbers)
    {
        int fovs = 8; //default is ML30SA1
        if(LidarType::ML30B1 == cal_lut.model)
            fovs = 3;
        else if(LidarType::ML30SA1 == cal_lut.model)
            fovs = 8;
        else if(LidarType::ML30SPlusA1 == cal_lut.model || LidarType::ML30SPlusB1 == cal_lut.model)
            fovs = 8;
        else if(LidarType::MLX == cal_lut.model)
            fovs = 3;
        else if(LidarType::MLXA1 == cal_lut.model)
            fovs = 3;
        else if(LidarType::MLXS == cal_lut.model)
            fovs = 3;
        else
            ;

        for(int i = 0; i < fovs * 2;i++)
            line_numbers[i] = 0;

        std::vector<PointCalibrationData>& data = cal_lut.data;
        for(int f = 0; f < fovs; f++)
        {
            int curr_line = 0;
            for(int g = 2; g < data.size() / fovs; g++)
            {
                int point_id = g * fovs + f;
                float pre_dif = data[point_id - fovs].azi - data[point_id - fovs * 2].azi; // pre - pre's pre
                float curr_dif = data[point_id].azi - data[point_id - fovs].azi; // curr - pre
                if((pre_dif * curr_dif) < 0.0)
                    curr_line++;

                line_numbers[point_id] = curr_line;
            }
        }
    }

    int LidarTools::SetDeviceRetroMode(std::string ip, bool en){
        /* connect to device */
        TcpClient client(1000, 1000, 1000);
        int ret = client.Connect(ip);
        if (ret)
        {
            ROS_ERROR_STREAM("Connect error: " << client.GetSysErrorCode());
            return -1;
        }

        /*Set device retro command*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xAB, (char)0x03, (char)0x00, (char)0x00 };
        if (en)
            set_cmd[2] = 0x01;
        else
            set_cmd[2] = 0x00;

        /* send */
        std::string cmd(set_cmd, send_len);
        if (client.SyncSend(cmd, send_len))
        {
            client.Close();
            return -1;
        }

        /* receive */
        const int recv_len = 4;
        std::string recv(recv_len, 'x');
        if(client.SyncRecv(recv, recv_len))
        {
            client.Close();
            return -1;
        }

        /* check ret */
        if (!CheckDeviceRet(recv))
        {
            client.Close();
            return -1;
        }

        client.Close();
        return 0;
    }

    int LidarTools::GetDeviceRetroMode(std::string ip, bool& en){

        /* connect to device */
        TcpClient client(1000, 1000, 1000);
        int ret = client.Connect(ip);
        if (ret)
        {
            ROS_ERROR_STREAM("Connect error: " << client.GetSysErrorCode());
            return -1;
        }

        /* send */
        const int send_len = 4;
        char cfg_read_cmd[send_len] = { (char)0xBA, (char)0x0B, (char)0x00, (char)0x00 };
        std::string cmd(cfg_read_cmd, 4);
        if (client.SyncSend(cmd, send_len))
        {
            client.Close();
            return -1;
        }

        /* receive ret */
        const int recv_len = 110;
        std::string recv(recv_len, 'x');
        if (client.SyncRecv(recv, 4))
        {
            client.Close();
            return -1;
        }

        /* check ret */
        if (!CheckDeviceRet(recv))
        {
            client.Close();
            return -1;
        }

        /* receive info */
        if (client.SyncRecv(recv, 106))
        {
            client.Close();
            return -1;
        }

        /* get retro state */
        unsigned char* header = (unsigned char*)recv.c_str();
        unsigned char retro = (*(header + 45));
        if ((0xFF == retro) || (0x00 == retro))
        {
            en = false;
        }
        else if (0x01 == retro)
        {
           en = true;
        }

        client.Close();
        return 0;
    }

    int LidarTools::RebootLidar(std::string ip){

        /* connect to device */
        TcpClient client(1000, 1000, 1000);
        int ret = client.Connect(ip);
        if (ret)
        {
            ROS_ERROR_STREAM("Connect error: " << client.GetSysErrorCode());
            return -1;
        }

        /* send */
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x0C, (char)0x00, (char)0x00 };
        std::string cmd(set_cmd, send_len);
        if (client.SyncSend(cmd, send_len))
        {
            client.Close();
            return -1;
        }

        /* receive ret */
        const int recv_len = 4;
        std::string recv(recv_len, 'x');
        if (client.SyncRecv(recv, recv_len))
        {
            client.Close();
            return -1;
        }

        /* check ret */
        if (!CheckDeviceRet(recv))
        {
            client.Close();
            return -1;
        }

        client.Close();
        return 0;
    }

    bool LidarTools::GetLidarOnlineState(std::string ip){
        /* connect to device */
        TcpClient client(1000, 1000, 1000);
        return client.Connect(ip) == 0;
    }

    uint16_t LidarTools::GetCheckSum(std::string str)
    {
        uint32_t check_sum = 0;
        uint8_t* pstr = (uint8_t* )str.data();
        for(int i = 0;i<str.size();i++)
            check_sum += *(pstr+i) &0xFF;

        uint16_t ret = check_sum & 0xFFFF;
        return ret;
    }
}
