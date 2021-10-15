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
#include <rclcpp/rclcpp.hpp>
#include <arpa/inet.h>
#include "tcp_client.h"

namespace zvision {


    bool LidarTools::CheckDeviceRet(std::string ret)
    {
        return (0x00 == ret[2]) && (0x00 == ret[3]);
    }

    int LidarTools::GetOnlineCalibrationData(std::string ip, CalibrationData& cal)
    {
        //std::string ec;
        //const int ppf = 256000; // points per frame, 256000 reserved
        const int ppk = 128; // points per cal udp packet
        //std::unique_ptr<float> angle_data(new float[ppf * 2]); // points( azimuh, elevation);
        //int packet_buffer_size = 1040 * (ppf / ppk) + 4; // 128 points in one packet, buffer reserved for ppf points.
        //std::unique_ptr<unsigned char> packet_data(new unsigned char[packet_buffer_size]);
        const int send_len = 4;
        char cal_cmd[send_len] = { (char)0xBA, (char)0x07, (char)0x00, (char)0x00 };
        std::string cmd(cal_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        cal.model = Unknown;
        TcpClient client(1000, 1000, 1000);
        int ret = client.Connect(ip);
        if (ret)
        {
            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Connect error: " << client.GetSysErrorCode());
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
            data_size = (6400 * 8 * 2);
            cal.model = ML30SA1;
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
            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Calibration packet identify error");
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
            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Check calibration data error, data is all 0x00.\n");
            client.Close();
            return -1;
        }
        if (0xFF == check_all_ff)
        {
            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Check calibration data error, data is all 0xFF.\n");
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
                std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Receive calibration data error\n");
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

        if (client.SyncRecv(recv, recv_len))
        {
            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Recv ack error.\n");
            client.Close();
            return -1;
        }
        //printf("recv ack ok.\n");
        if (!CheckDeviceRet(recv))
        {
            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Check ack error.\n");
            client.Close();
            return -1;
        }

        if(data_size != data.size())
        {
            std::cout << "error" << std::endl;//ROS_ERROR("Calbration data size [%6lu] is not valid, [%6u] wanted.\n", data.size(), data_size);
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
                std::cout << "error" << std::endl;//ROS_ERROR("Calbration file line size [%d] is not valid.\n", lines.size());
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
                #if 0
                else if (("ML30S_160_1_2" == mode_str) || ("ML30S_160_1_4" == mode_str))
                {
                    int group = 3200;
                    if ("ML30S_160_1_2" == mode_str)
                    {
                        group = 3200;
                        cal.scan_mode = ScanMode::ScanML30SA1_160_1_2;
                    }
                    else
                    {
                        group = 1600;
                        cal.scan_mode = ScanMode::ScanML30SA1_160_1_4;
                    }

                    if ((lines.size() - curr) < group)//check the file lines
                        return -1;

                    cal.data.resize(group * 8 * 2);
                    for (int i = 0; i < group; i++)
                    {
                        const int column = 17;

                        std::vector<std::string>& datas = lines[i + curr];
                        if (datas.size() != column)
                        {
                            ret = InvalidContent;
                            break;
                        }
                        for (int j = 1; j < column; j++)
                        {
                            cal.data[i * 16 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                        }
                    }
                    cal.device_type = DeviceType::LidarML30SA1;
                    cal.description = "";
                    for (int i = 0; i < curr; i++)
                    {
                        for (int j = 0; j < lines[i].size(); j++)
                            cal.description += lines[i][j];
                        cal.description += "\n";
                    }
                }
                #endif
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
                            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Resolve calibration file data error.");
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
                            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Resolve calibration file data error.");
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
                            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Resolve calibration file data error.");
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
                            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Resolve calibration file data error.");
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
                    std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Invalid calibration file length.");
                }
            }

            return ret;
        }
        else
        {
            std::cout << "error" << std::endl;//ROS_ERROR_STREAM("Open calibration file error.");
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

}
