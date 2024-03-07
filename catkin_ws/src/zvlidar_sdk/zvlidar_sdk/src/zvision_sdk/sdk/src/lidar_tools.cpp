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
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "lidar_tools.h"
#include "client.h"
#include "loguru.hpp"
#include "packet.h"
#include "print.h"

#include <rapidjson/document.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <math.h>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
static const int EXCITON_COMPDATALEN = 1536;

namespace zvision {
LidarTools::LidarTools(std::string lidar_ip, int con_timeout, int send_timeout,
                       int recv_timeout)
    : client_(new TcpClient(con_timeout, send_timeout, recv_timeout)),
      device_ip_(lidar_ip), conn_ok_(false) {}

LidarTools::~LidarTools() { this->DisConnect(); }

int LidarTools::CaptureLidarsHeartbeat(std::vector<std::string> &lidars,
                                       int scan_time) {
  const auto before = std::chrono::system_clock::now();

  const int heart_beat_port = 55000;
  UdpReceiver recv(heart_beat_port, 100);
  std::string data;
  int len = 0;
  uint32_t ip = 0;
  int ret = 0;
  lidars.clear();

  LOG_F(2, "\nScan device on the heart beat port %d for %d second(s).",
        heart_beat_port, scan_time);
  while (1) {
    if (0 != (ret = recv.SyncRecv(data, len, ip))) {
      LOG_F(ERROR, "Scan device error, receive failed ret = %d.", ret);
      break;
    }

    if (is_lidar_heartbeat_packet(data)) {
      std::string ip_string = IpToString(ip);
      lidars.push_back(ip_string);
    }

    // time elapse
    std::chrono::duration<double> duration =
        std::chrono::system_clock::now() - before;
    if (duration.count() >= scan_time)
      break;
  }

  // remove the same value
  // https://stackoverflow.com/questions/1041620/whats-the-most-efficient-way-to-erase-duplicates-and-sort-a-vector
  std::set<std::string> set;
  unsigned size = lidars.size();
  for (unsigned i = 0; i < size; ++i)
    set.insert(lidars[i]);
  lidars.assign(set.begin(), set.end());

  return 0;
}

int LidarTools::ScanDevice(std::vector<DeviceConfigurationInfo> &device_list,
                           int scan_time) {
  std::vector<std::string> lidars;
  CaptureLidarsHeartbeat(lidars, scan_time);
  int ret = 0;
  for (unsigned int i = 0; i < lidars.size(); ++i) {
    LidarTools config(lidars[0], 1000);
    DeviceConfigurationInfo info;
    if (0 != (ret = config.QueryDeviceConfigurationInfo(info))) {
      LOG_F(ERROR, "Scan device error, query device info failed, ret = %d.",
            ret);
      break;
    }
    device_list.push_back(info);
  }
  return ret;
}

int LidarTools::ScanML30sPlusB1Device(
    std::vector<DeviceConfigurationInfo> &device_list, int scan_time) {
  std::vector<std::string> lidars;
  CaptureLidarsHeartbeat(lidars, scan_time);
  int ret = 0;
  for (unsigned int i = 0; i < lidars.size(); ++i) {
    LidarTools config(lidars[0], 1000);
    DeviceConfigurationInfo info;
    if (0 != (ret = config.QueryML30sPlusB1DeviceConfigurationInfo(info))) {
      LOG_F(ERROR, "Scan device error, query device info failed, ret = %d.",
            ret);
      break;
    }
    device_list.push_back(info);
  }
  return ret;
}

int LidarTools::ReadCalibrationData(std::string filename,
                                    CalibrationData &cal) {

  const int file_min_lines = 4;
  std::ifstream file;
  file.open(filename, std::ios::in);
  std::string line;
  if (!file.is_open())
    return OpenFileError;

  // read all lines to vector
  std::vector<std::vector<std::string>> lines;
  while (std::getline(file, line)) {
    if (line.size() > 0) {
      std::istringstream iss(line);
      std::vector<std::string> datas;
      std::string data;
      while (iss >> data) {
        datas.push_back(data);
      }
      lines.push_back(datas);
    }
  }
  file.close();
  //
  if (lines.size() < file_min_lines)
    return InvalidContent;

  // filter #
  int curr = 0;
  for (auto &au : lines) {
    if ('#' != au[0][0])
      break;
    curr++;
  }

  // read version info
  std::string version_str;
  if ((lines[curr].size() >= 2) && (lines[curr][0] == "VERSION")) {
    version_str = lines[curr][lines[curr].size() - 1];
    curr++;
  }

  // read scan mode
  std::string mode_str;
  if ((lines[curr].size() >= 2) && (lines[curr][0] == "Mode")) {
    mode_str = lines[curr][lines[curr].size() - 1];
    curr++;
  }
  int ret = 0;
  
  if (mode_str.size()) {
    if ("MLX_A1_190" == mode_str) {
      if ((lines.size() - curr) < 38000) // check the file lines
        return InvalidContent;

      cal.data.resize(38000 * 3 * 2);
      for (int i = 0; i < 38000; i++) {
        const int column = 7;

        std::vector<std::string> &datas = lines[i + curr];
        if (datas.size() != column) {
          ret = InvalidContent;
          break;
        }
        for (int j = 1; j < column; j++) {
          cal.data[i * 6 + j - 1] =
              static_cast<float>(std::atof(datas[j].c_str()));
        }
      }
      cal.device_type = DeviceType::LidarMLX;
      cal.scan_mode = ScanMode::ScanMLX_190;
      cal.description = "";
      for (int i = 0; i < curr; i++) {
        for (int j = 0; j < lines[i].size(); j++)
          cal.description += lines[i][j];
        cal.description += "\n";
      }
    } else if (("ML30S_160_1_2" == mode_str) || ("ML30S_160_1_4" == mode_str)) {
      int group = 3200;
      if ("ML30S_160_1_2" == mode_str) {
        group = 3200;
        cal.scan_mode = ScanMode::ScanML30SA1_160_1_2;
      } else {
        group = 1600;
        cal.scan_mode = ScanMode::ScanML30SA1_160_1_4;
      }

      if ((lines.size() - curr) < group) // check the file lines
        return InvalidContent;

      cal.data.resize(group * 8 * 2);
      for (int i = 0; i < group; i++) {
        const int column = 17;

        std::vector<std::string> &datas = lines[i + curr];
        if (datas.size() != column) {
          ret = InvalidContent;
          break;
        }
        for (int j = 1; j < column; j++) {
          cal.data[i * 16 + j - 1] =
              static_cast<float>(std::atof(datas[j].c_str()));
        }
      }
      cal.device_type = DeviceType::LidarML30SA1;
      cal.description = "";
      for (int i = 0; i < curr; i++) {
        for (int j = 0; j < lines[i].size(); j++)
          cal.description += lines[i][j];
        cal.description += "\n";
      }
    } else if ("MLXs_180" == mode_str) {
      if ((lines.size() - curr) < 36000) // check the file lines
        return InvalidContent;

      cal.data.resize(36000 * 3 * 2);
      for (int i = 0; i < 36000; i++) {
        const int column = 7;

        std::vector<std::string> &datas = lines[i + curr];
        if (datas.size() != column) {
          ret = InvalidContent;
          break;
        }
        for (int j = 1; j < column; j++) {
          cal.data[i * 6 + j - 1] =
              static_cast<float>(std::atof(datas[j].c_str()));
        }
      }
      cal.device_type = DeviceType::LidarMLX;
      cal.scan_mode = ScanMode::ScanMLXS_180;
      cal.description = "";
      for (int i = 0; i < curr; i++) {
        for (int j = 0; j < lines[i].size(); j++)
          cal.description += lines[i][j];
        cal.description += "\n";
      }
    } else if ("ML30SPlus_160" == mode_str || "ML30SPlus_160_1_2" == mode_str ||
               "ML30SPlus_160_1_4" == mode_str) {
      // for ml30sa1Plus
      int cali_len = 6400;
      cal.device_type = DeviceType::LidarMl30SA1Plus;
      cal.scan_mode = ScanMode::ScanML30SA1Plus_160;
      cal.description = "";
      if ("ML30SPlus_160_1_2" == mode_str) {
        cali_len = 3200;
        cal.scan_mode = ScanMode::ScanML30SA1Plus_160_1_2;
      } else if ("ML30SPlus_160_1_4" == mode_str) {
        cali_len = 1600;
        cal.scan_mode = ScanMode::ScanML30SA1Plus_160_1_4;
      }

      if ((lines.size() - curr) < cali_len)
        return InvalidContent;

      cal.data.resize(cali_len * 8 * 2);
      const int column = 17;
      const int lpos = cal.data.size() / 2;
      for (int i = 0; i < cali_len; i++) {
        std::vector<std::string> &datas = lines[i + curr];
        if (datas.size() != column) {
          ret = InvalidContent;
          break;
        }
        
        set_ml30splus_b1_ep_mode_enable(true);

        if (zvision::is_ml30splus_b1_ep_mode_enable()) 
        {
          // for line with fov 0,1,2,3,4,5,6,7
          for (int j = 1; j < column; j++) {
            cal.data[i * 16 + j - 1] =
                static_cast<float>(std::atof(datas[j].c_str()));
          }
        } 
        else 
        {
          // for line with fov 0,1,2,3,4,5,6,7
          for (int j = 1; j < column; j++) {
            if (j <= column / 2) // for H view
              cal.data[i * 8 + j - 1] =
                  static_cast<float>(std::atof(datas[j].c_str()));
            else // for L view
              cal.data[i * 8 + j - 1 - column / 2 + lpos] =
                  static_cast<float>(std::atof(datas[j].c_str()));
          }
        }
      }

      for (int i = 0; i < curr; i++) {
        for (int j = 0; j < lines[i].size(); j++)
          cal.description += lines[i][j];
        cal.description += "\n";
      }
    } else {
      return InvalidContent;
    }
  } else {
    if (10000 == lines.size()) {
      cal.data.resize(60000);
      for (int i = 0; i < 10000; i++) {
        const int column = 7;

        std::vector<std::string> &datas = lines[i];
        if (datas.size() != column) {
          ret = InvalidContent;
          break;
        }
        for (int j = 1; j < column; j++) {
          int fov = (j - 1) % 3;
          if (0 == ((j - 1) / 3)) // azimuth
          {
            cal.data[i * 6 + fov * 2] =
                static_cast<float>(std::atof(datas[j].c_str()));
          } else // elevation
          {
            cal.data[i * 6 + fov * 2 + 1] =
                static_cast<float>(std::atof(datas[j].c_str()));
          }
        }
      }
      cal.device_type = DeviceType::LidarML30B1;
      cal.scan_mode = ScanMode::ScanML30B1_100;
    } else if (6400 == lines.size()) {
      
      cal.data.resize(6400 * 8 * 2);
      for (int i = 0; i < 6400; i++) {
        const int column = 17;

        std::vector<std::string> &datas = lines[i];
        if (datas.size() != column) {
          ret = InvalidContent;
          break;
        }
        for (int j = 1; j < column; j++) {
          cal.data[i * 16 + j - 1] =
              static_cast<float>(std::atof(datas[j].c_str()));
        }
      }
      cal.device_type = DeviceType::LidarML30SA1;
      cal.scan_mode = ScanMode::ScanML30SA1_160;
    } else if (7600 == lines.size()) {
      cal.data.resize(7200 * 8 * 2);
      for (int i = 0; i < 7200; i++) {
        const int column = 17;

        std::vector<std::string> &datas = lines[i];
        if (datas.size() != column) {
          ret = InvalidContent;
          break;
        }
        for (int j = 1; j < column; j++) {
          cal.data[i * 16 + j - 1] =
              static_cast<float>(std::atof(datas[j].c_str()));
        }
      }
      cal.device_type = DeviceType::LidarML30SA1;
      cal.scan_mode = ScanMode::ScanML30SA1_190;
    } else if (32000 == lines.size()) {
      cal.data.resize(32000 * 3 * 2);
      for (int i = 0; i < 32000; i++) {
        const int column = 7;

        std::vector<std::string> &datas = lines[i];
        if (datas.size() != column) {
          ret = InvalidContent;
          break;
        }
        for (int j = 1; j < column; j++) {
          cal.data[i * 6 + j - 1] =
              static_cast<float>(std::atof(datas[j].c_str()));
        }
      }
      cal.device_type = DeviceType::LidarMLX;
      cal.scan_mode = ScanMode::ScanMLX_160;
    } else if (4800 == lines.size()) {
      cal.data.resize(4800 * 3 * 2);
      for (int i = 0; i < 4800; i++) {
        const int column = 7;

        std::vector<std::string> &datas = lines[i];
        if (datas.size() != column) {
          ret = InvalidContent;
          break;
        }
        for (int j = 1; j < column; j++) {
          cal.data[i * 6 + j - 1] =
              static_cast<float>(std::atof(datas[j].c_str()));
        }
      }
      cal.device_type = DeviceType::LidarMLYB;
      cal.scan_mode = ScanMode::ScanMLYB_190;
    } else if (38000 == lines.size()) {
      cal.data.resize(38000 * 3 * 2);
      for (int i = 0; i < 38000; i++) {
        const int column = 7;

        std::vector<std::string> &datas = lines[i];
        if (datas.size() != column) {
          ret = InvalidContent;
          break;
        }
        for (int j = 1; j < column; j++) {
          cal.data[i * 6 + j - 1] =
              static_cast<float>(std::atof(datas[j].c_str()));
        }
      }
      cal.device_type = DeviceType::LidarMLYA;
      cal.scan_mode = ScanMode::ScanMLYA_190;
    } else {
      cal.device_type = DeviceType::LidarUnknown;
      cal.scan_mode = ScanMode::ScanUnknown;
      ret = NotSupport;
    }
  }

  return ret;
}

int LidarTools::ReadCalibrationData(
    std::string filename, CalibrationDataSinCosTable &cal_cos_sin_lut) {
  CalibrationData cal;
  int ret = 0;
  if (0 != (ret = LidarTools::ReadCalibrationData(filename, cal))) {
    return ret;
  } else {
    LidarTools::ComputeCalibrationSinCos(cal, cal_cos_sin_lut);
    return 0;
  }
}

int LidarTools::ExportCalibrationData(CalibrationData &cal,
                                      std::string filename) {
  if ((ScanMode::ScanML30SA1_160 == cal.scan_mode) ||
      (ScanMode::ScanML30SA1_190 == cal.scan_mode)) {
    std::fstream outfile;
    outfile.open(filename, std::ios::out);
    const int data_in_line = 16;
    if (outfile.is_open()) {
      outfile.setf(std::ios::fixed, std::ios::floatfield);
      outfile.precision(3);
      for (unsigned int i = 0; i < cal.data.size(); i++) {
        if (0 == (i % data_in_line)) {
          if (i > 0)
            outfile << "\n";
          outfile << i / data_in_line + 1;
        }
        outfile << " " << cal.data[i];
      }
      outfile.close();
      return 0;
    } else {
      return OpenFileError;
    }
  } else if ((ScanMode::ScanML30SA1_160_1_2 == cal.scan_mode) ||
             (ScanMode::ScanML30SA1_160_1_4 == cal.scan_mode)) {
    std::fstream outfile;
    outfile.open(filename, std::ios::out);
    const int data_in_line = 16;
    if (outfile.is_open()) {
      outfile << "# file version : ML30S.cal_v0.4\n";
      outfile << "VERSION 0.1\n";
      if (ScanMode::ScanML30SA1_160_1_2 == cal.scan_mode)
        outfile << "Mode ML30S_160_1_2\n";
      else
        outfile << "Mode ML30S_160_1_4\n";

      outfile.setf(std::ios::fixed, std::ios::floatfield);
      outfile.precision(3);
      for (unsigned int i = 0; i < cal.data.size(); i++) {
        if (0 == (i % data_in_line)) {
          if (i > 0)
            outfile << "\n";
          outfile << i / data_in_line + 1;
        }
        outfile << " " << cal.data[i];
      }
      outfile.close();
      return 0;
    } else {
      return OpenFileError;
    }
  } else if (ScanML30SA1Plus_160 == cal.scan_mode ||
             ScanML30SA1Plus_160_1_2 == cal.scan_mode ||
             ScanML30SA1Plus_160_1_4 == cal.scan_mode) {
    std::fstream outfile;
    outfile.open(filename, std::ios::out);
    const int data_in_line = 16;
    if (outfile.is_open()) {
      outfile << "# file version:ML30SPlus.cal_v0.1";
      outfile << "VERSION 0.1\n";
      if (ScanML30SA1Plus_160 == cal.scan_mode)
        outfile << "Mode ML30SPlus_160\n";
      else if (ScanML30SA1Plus_160_1_2 == cal.scan_mode)
        outfile << "Mode ML30SPlus_160_1_2\n";
      else
        outfile << "Mode ML30SPlus_160_1_4\n";

      outfile.setf(std::ios::fixed, std::ios::floatfield);
      outfile.precision(3);
      int lpos = cal.data.size() / 2;
      for (unsigned int i = 0; i < cal.data.size(); i++) {
        if (0 == (i % data_in_line)) {
          if (i > 0)
            outfile << "\n";
          outfile << i / data_in_line + 1;
        }

        if (zvision::is_ml30splus_b1_ep_mode_enable()) {
          outfile << " " << cal.data[i];
        } else {
          int idx = i % 16;
          int grp = i / 16;
          if (idx < 8) {
            outfile << " " << cal.data[idx + grp * 8];
          } else {
            outfile << " " << cal.data[(idx - 8) + grp * 8 + lpos];
          }
        }
      }
      outfile.close();
      return 0;
    } else {
      return OpenFileError;
    }
  } else if (ScanMode::ScanML30B1_100 == cal.scan_mode) {
    std::fstream outfile;
    outfile.open(filename, std::ios::out);
    if (outfile.is_open()) {
      outfile.setf(std::ios::fixed, std::ios::floatfield);
      outfile.precision(3);
      int rows = 10000;
      for (int i = 0; i < rows; i++) {
        outfile << i + 1 << " ";
        outfile << cal.data[i * 6 + 0] << " ";
        outfile << cal.data[i * 6 + 2] << " ";
        outfile << cal.data[i * 6 + 4] << " ";
        outfile << cal.data[i * 6 + 1] << " ";
        outfile << cal.data[i * 6 + 3] << " ";
        outfile << cal.data[i * 6 + 5];
        if (i < (rows - 1))
          outfile << "\n";
      }
      outfile.close();
      return 0;
    } else {
      return OpenFileError;
    }
  } else if (ScanMode::ScanMLX_160 == cal.scan_mode) {
    std::fstream outfile;
    outfile.open(filename, std::ios::out);
    if (outfile.is_open()) {
      outfile.setf(std::ios::fixed, std::ios::floatfield);
      outfile.precision(3);
      int rows = 32000;
      for (int i = 0; i < rows; i++) {
        outfile << i + 1 << " ";
        outfile << cal.data[i * 6 + 0] << " ";
        outfile << cal.data[i * 6 + 1] << " ";
        outfile << cal.data[i * 6 + 2] << " ";
        outfile << cal.data[i * 6 + 3] << " ";
        outfile << cal.data[i * 6 + 4] << " ";
        outfile << cal.data[i * 6 + 5];
        if (i < (rows - 1))
          outfile << "\n";
      }
      outfile.close();
      return 0;
    } else {
      return OpenFileError;
    }
  } else if (ScanMode::ScanMLX_190 == cal.scan_mode) {
    std::fstream outfile;
    outfile.open(filename, std::ios::out);
    if (outfile.is_open()) {
      outfile.setf(std::ios::fixed, std::ios::floatfield);
      outfile.precision(3);
      outfile << cal.description;
      int rows = 38000;
      for (int i = 0; i < rows; i++) {
        outfile << i + 1 << " ";
        outfile << cal.data[i * 6 + 0] << " ";
        outfile << cal.data[i * 6 + 1] << " ";
        outfile << cal.data[i * 6 + 2] << " ";
        outfile << cal.data[i * 6 + 3] << " ";
        outfile << cal.data[i * 6 + 4] << " ";
        outfile << cal.data[i * 6 + 5];
        if (i < (rows - 1))
          outfile << "\n";
      }
      outfile.close();
      return 0;
    } else {
      return OpenFileError;
    }
  } else if (ScanMode::ScanMLXS_180 == cal.scan_mode) {
    std::fstream outfile;
    outfile.open(filename, std::ios::out);
    if (outfile.is_open()) {
      outfile << "# file version:MLXs.cal_v0.0\n";
      outfile << "VERSION 0.1\n";
      outfile << "Mode MLXs_180\n";

      outfile.setf(std::ios::fixed, std::ios::floatfield);
      outfile.precision(3);
      outfile << cal.description;
      int rows = 36000;
      for (int i = 0; i < rows; i++) {
        outfile << i + 1 << " ";
        outfile << cal.data[i * 6 + 0] << " ";
        outfile << cal.data[i * 6 + 1] << " ";
        outfile << cal.data[i * 6 + 2] << " ";
        outfile << cal.data[i * 6 + 3] << " ";
        outfile << cal.data[i * 6 + 4] << " ";
        outfile << cal.data[i * 6 + 5];
        if (i < (rows - 1))
          outfile << "\n";
      }
      outfile.close();
      return 0;
    } else {
      return OpenFileError;
    }
  } else {
    return NotSupport;
  }
}

void LidarTools::ComputeCalibrationSinCos(
    CalibrationData &cal, CalibrationDataSinCosTable &cal_cos_sin_lut) {
  cal_cos_sin_lut.device_type = cal.device_type;
  cal_cos_sin_lut.scan_mode = cal.scan_mode;
  cal_cos_sin_lut.description = cal.description;
  cal_cos_sin_lut.data.resize(cal.data.size() / 2);

  if (ScanMode::ScanML30B1_100 == cal.scan_mode) {
    for (unsigned int i = 0; i < cal.data.size() / 2; ++i) {
      float azi = static_cast<float>(cal.data[i * 2] / 180.0 * 3.1416);
      float ele = static_cast<float>(cal.data[i * 2 + 1] / 180.0 * 3.1416);

      CalibrationDataSinCos &point_cal = cal_cos_sin_lut.data[i];
      point_cal.cos_ele = std::cos(ele);
      point_cal.sin_ele = std::sin(ele);
      point_cal.cos_azi = std::cos(azi);
      point_cal.sin_azi = std::sin(azi);
      point_cal.azi = azi;
      point_cal.ele = ele;
    }
  } else if ((ScanMode::ScanML30SA1_160 == cal.scan_mode) ||
             (ScanMode::ScanML30SA1_190 == cal.scan_mode) ||
             (ScanMode::ScanML30SA1_160_1_2 == cal.scan_mode) ||
             (ScanMode::ScanML30SA1_160_1_4 == cal.scan_mode)) {
    const int start = 8;
    int fov_index[start] = {0, 6, 1, 7, 2, 4, 3, 5};
    for (unsigned int i = 0; i < cal.data.size() / 2; ++i) {
      int start_number = i % start;
      int group_number = i / start;
      int point_numer = group_number * start + fov_index[start_number];
      float azi =
          static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
      float ele =
          static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

      CalibrationDataSinCos &point_cal = cal_cos_sin_lut.data[i];
      point_cal.cos_ele = std::cos(ele);
      point_cal.sin_ele = std::sin(ele);
      point_cal.cos_azi = std::cos(azi);
      point_cal.sin_azi = std::sin(azi);
      point_cal.azi = azi;
      point_cal.ele = ele;
    }
  } else if (ScanMode::ScanML30SA1Plus_160 == cal.scan_mode) {
    const int start = 4;
    int fov_index[start] = {0, 1, 2, 3};
    for (unsigned int i = 0; i < cal.data.size() / 2; ++i) {
      int start_number = i % start;
      int group_number = i / start;
      int point_numer = group_number * start + fov_index[start_number];
      float azi =
          static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
      float ele =
          static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

      CalibrationDataSinCos &point_cal = cal_cos_sin_lut.data[i];
      point_cal.cos_ele = std::cos(ele);
      point_cal.sin_ele = std::sin(ele);
      point_cal.cos_azi = std::cos(azi);
      point_cal.sin_azi = std::sin(azi);
      point_cal.azi = azi;
      point_cal.ele = ele;
    }

  } else if ((ScanMode::ScanMLX_160 == cal.scan_mode) ||
             (ScanMode::ScanMLX_190 == cal.scan_mode) ||
             (ScanMode::ScanMLXS_180 == cal.scan_mode)) {
    const int start = 3;
    int fov_index[start] = {0, 1, 2};
    for (unsigned int i = 0; i < cal.data.size() / 2; ++i) {
      int start_number = i % start;
      int group_number = i / start;
      int point_numer = group_number * start + fov_index[start_number];
      float azi =
          static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
      float ele =
          static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

      CalibrationDataSinCos &point_cal = cal_cos_sin_lut.data[i];
      point_cal.cos_ele = std::cos(ele);
      point_cal.sin_ele = std::sin(ele);
      point_cal.cos_azi = std::cos(azi);
      point_cal.sin_azi = std::sin(azi);
      point_cal.azi = azi;
      point_cal.ele = ele;
    }
  } else if (ScanMode::ScanMLYA_190 == cal.scan_mode) {
    const int start = 3;
    int fov_index[start] = {2, 1, 0};
    for (unsigned int i = 0; i < cal.data.size() / 2; ++i) {
      int start_number = i % start;
      int group_number = i / start;
      int point_numer = group_number * start + fov_index[start_number];
      float azi =
          static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
      float ele =
          static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

      CalibrationDataSinCos &point_cal = cal_cos_sin_lut.data[i];
      point_cal.cos_ele = std::cos(ele);
      point_cal.sin_ele = std::sin(ele);
      point_cal.cos_azi = std::cos(azi);
      point_cal.sin_azi = std::sin(azi);
      point_cal.azi = azi;
      point_cal.ele = ele;
    }
  } else if (ScanMode::ScanMLYB_190 == cal.scan_mode) {
    const int start = 3;
    int fov_index[start] = {2, 1, 0};
    for (unsigned int i = 0; i < cal.data.size() / 2; ++i) {
      int start_number = i % start;
      int group_number = i / start;
      int point_numer = group_number * start + fov_index[start_number];
      float azi =
          static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
      float ele =
          static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

      CalibrationDataSinCos &point_cal = cal_cos_sin_lut.data[i];
      point_cal.cos_ele = std::cos(ele);
      point_cal.sin_ele = std::sin(ele);
      point_cal.cos_azi = std::cos(azi);
      point_cal.sin_azi = std::sin(azi);
      point_cal.azi = azi;
      point_cal.ele = ele;
    }
  } else {
  }
}

int LidarTools::QueryDeviceHdcpNetAddr(std::string &addrs) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Read device mac info*/
  const int send_len = 4;
  char sn_read_cmd[send_len] = {(char)0xBA, (char)0x0B, (char)0x03, (char)0x00};
  std::string cmd(sn_read_cmd, 4);

  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  const int recv_len = 20;
  std::string recv(recv_len, 'x');
  if (client_->SyncRecv(recv, 4)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // check
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  // Assuming read failure while len bigger than 16.
  int len = client_->GetAvailableBytesLen();
  if (len > 16) {
    // clear receive buffer
    std::string temp(len, 'x');
    client_->SyncRecv(temp, len);
    return TcpRecvTimeout;
  }

  if (client_->SyncRecv(recv, 16)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  addrs = recv.substr(0, 16);

  return 0;
}

int LidarTools::QueryDeviceFactoryMac(std::string &mac) {

  if (!CheckConnection())
    return TcpConnTimeout;

  /*Read device mac info*/
  const int send_len = 4;
  char sn_read_cmd[send_len] = {(char)0xBA, (char)0x0B, (char)0x02, (char)0x00};
  std::string cmd(sn_read_cmd, 4);

  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  const int recv_len = 8;
  std::string recv(recv_len, 'x');
  if (client_->SyncRecv(recv, 4)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // check
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  // Assuming read failure while length is not 8.
  int len = client_->GetAvailableBytesLen();
  if (len != 8) {
    // clear receive buffer
    std::string temp(len, 'x');
    client_->SyncRecv(temp, len);
    return TcpRecvTimeout;
  }

  if (client_->SyncRecv(recv, 8)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  std::string flag = recv.substr(0, 2);
  if (flag.compare("MA") != 0) {
    return NotMatched;
  }

  std::string addr = recv.substr(2, 6);
  // if (addr[0]!= (char)0xF8 || addr[1] != (char)0xA9 || addr[2] != (char)0x1F)
  // { 	return -1;
  // }
  char cmac[256] = "";
  sprintf_s(cmac, "%02X%02X%02X%02X%02X%02X", (uint8_t)addr[0],
            (uint8_t)addr[1], (uint8_t)addr[2], (uint8_t)addr[3],
            (uint8_t)addr[4], (uint8_t)addr[5]);
  mac = std::string(cmac);
  return 0;
}

int LidarTools::QueryDeviceSnCode(std::string &sn) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Read version info*/
  const int send_len = 4;
  char sn_read_cmd[send_len] = {(char)0xBA, (char)0x08, (char)0x00, (char)0x00};
  std::string cmd(sn_read_cmd, 4);

  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  const int recv_len = 21;
  std::string recv(recv_len, 'x');
  if (client_->SyncRecv(recv, 4)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  if (client_->SyncRecv(recv, 17)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  {
    int len = client_->GetAvailableBytesLen();
    if (len > 0) {
      std::string recvLeft(len, 'x');
      if (client_->SyncRecv(recvLeft, len)) {
        DisConnect();
      } else {
        sn = recv.substr(0, 17) + recvLeft;
        return 0;
      }
    }
  }

  sn = recv.substr(0, 17);
  return 0;
}

int LidarTools::QueryDeviceFirmwareVersion(FirmwareVersion &version) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Read version info*/
  const int send_len = 4;
  char bv_read_cmd[send_len] = {(char)0xBA, (char)0x02, (char)0x01, (char)0x00};
  std::string bv_cmd(bv_read_cmd, 4);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(bv_cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv version data
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  memcpy(&(version.boot_version), recv.c_str(), 4);

  char kv_read_cmd[send_len] = {(char)0xBA, (char)0x02, (char)0x02, (char)0x00};
  std::string kv_cmd(kv_read_cmd, 4);

  if (client_->SyncSend(kv_cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv version data
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  memcpy(&(version.kernel_version), recv.c_str(), 4);

  return 0;
}

int LidarTools::QueryDeviceTemperature(float &PS, float &PL) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Read temperature info*/
  const int send_len = 4;
  char te_read_cmd[send_len] = {(char)0xAB, (char)0x06, (char)0x00, (char)0x00};
  std::string cmd(te_read_cmd, 4);

  const int recv_len = 12;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  NetworkToHost((const unsigned char *)recv.c_str() + 4, (char *)&PS);
  NetworkToHost((const unsigned char *)recv.c_str() + 8, (char *)&PL);

  return 0;
}

int LidarTools::QueryDeviceConfigurationInfo(DeviceConfigurationInfo &info) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Read configuration info*/
  const int send_len = 4;
  char cfg_read_cmd[send_len] = {(char)0xBA, (char)0x0B, (char)0x00,
                                 (char)0x00};
  std::string cmd(cfg_read_cmd, 4);

  const int recv_len = 110;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, 4)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  if (client_->SyncRecv(recv, 106)) // recv configuration data
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  unsigned char *header = (unsigned char *)recv.c_str();
  memcpy(info.version.boot_version, header + 0, 4);
  memcpy(info.version.kernel_version, header + 16, 4);
  ResolveIpString(header + 32, info.device_ip);
  ResolvePort(header + 36, info.destination_port);

  unsigned char time_sync = (*(header + 40));
  info.time_sync = TimestampType::TimestampUnknown;
  if ((0xFF == time_sync) || (0x01 == time_sync)) {
    info.time_sync = TimestampType::TimestampPtp;
  } else if (0x02 == time_sync) {
    info.time_sync = TimestampType::TimestampPpsGps;
  }

  ResolveIpString(header + 41, info.destination_ip);

  unsigned char retro = (*(header + 45));
  info.retro_enable = RetroMode::RetroUnknown;
  if ((0xFF == retro) || (0x00 == retro)) {
    info.retro_enable = RetroMode::RetroDisable;
  } else if (0x01 == retro) {
    info.retro_enable = RetroMode::RetroEnable;
  }

  ResolveIpString(header + 46, info.subnet_mask);
  ResolveMacAddress(header + 100, info.device_mac);

  // config mac
  ResolveMacAddress(header + 50, info.config_mac);

  // phase offset
  NetworkToHost(header + 56, (char *)&info.phase_offset);

  // echo mode
  unsigned char echo_mode = (*(header + 60));
  info.echo_mode = EchoMode::EchoUnknown;
  if (0x01 == echo_mode)
    info.echo_mode = EchoSingleFirst;
  else if (0x02 == echo_mode)
    info.echo_mode = EchoSingleStrongest;
  else if (0x04 == echo_mode)
    info.echo_mode = EchoSingleLast;
  else if (0x03 == echo_mode)
    info.echo_mode = EchoDoubleFirstStrongest;
  else if (0x05 == echo_mode)
    info.echo_mode = EchoDoubleFirstLast;
  else if (0x06 == echo_mode)
    info.echo_mode = EchoDoubleStrongestLast;

  // phase offset enable
  unsigned char phase_offset_enable = (*(header + 61));
  if (0x00 == phase_offset_enable)
    info.phase_offset_mode = PhaseOffsetDisable;
  else if (0x01 == phase_offset_enable)
    info.phase_offset_mode = PhaseOffsetEnable;
  else
    info.phase_offset_mode = PhaseOffsetUnknown;

  // retro param
  unsigned char retro_param_1 = (*(header + 62));
  unsigned char retro_param_2 = (*(header + 63));
  info.retro_param_1_ref_min = retro_param_1;
  info.retro_param_2_point_percent = retro_param_2;

  // calibration data send status, 0 disable, 1 enable
  unsigned char cal_send_enable = (*(header + 64));
  if (0x00 == cal_send_enable)
    info.cal_send_mode = CalSendDisable;
  else if (0x01 == cal_send_enable)
    info.cal_send_mode = CalSendEnable;
  else
    info.cal_send_mode = CalSendUnknown;

  // downsample mode
  unsigned char downsample_flag = (*(header + 65));
  if (0x00 == downsample_flag)
    info.downsample_mode = DownsampleNone;
  else if (0x01 == downsample_flag)
    info.downsample_mode = Downsample_1_2;
  else if (0x02 == downsample_flag)
    info.downsample_mode = Downsample_1_4;
  else
    info.downsample_mode = DownsampleUnknown;

  // hard diagnostic control
  NetworkToHost(header + 82, (char *)&info.hard_diag_ctrl);

  // dhcp enable
  unsigned char value = (*(header + 86));
  if (value == 0x00)
    info.dhcp_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.dhcp_enable = StateMode::StateEnable;
  else {
    // info.dhcp_enable = StateMode::StateUnknown;
    //  Adapt to earlier versions
    info.dhcp_enable = StateMode::StateDisable;
  }

  // lidar gateway addr
  ResolveIpString(header + 87, info.gateway_addr);

  // delete point switch
  value = (*(header + 91));
  if (value == 0x00)
    info.delete_point_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.delete_point_enable = StateMode::StateEnable;
  else
    info.delete_point_enable = StateMode::StateUnknown;

  // adhesion switch
  value = (*(header + 92));
  if (value == 0x00)
    info.adhesion_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.adhesion_enable = StateMode::StateEnable;
  else
    info.adhesion_enable = StateMode::StateUnknown;

  // retro gray low/ threshold
  info.algo_param.retro_gray_low_threshold = (*(header + 93));
  info.algo_param.retro_gray_high_threshold = (*(header + 94));
  info.delete_point_mode = (*(header + 99));

  // clear recv buffer
  int len = client_->GetAvailableBytesLen();
  if (len > 0) {
    std::string temp(len, 'x');
    client_->SyncRecv(temp, len);
  }

  // sn code and device type
  std::string sn = "Unknown";
  info.serial_number = "Unknown";
  info.device = DeviceType::LidarUnknown;
  if (QueryDeviceSnCode(sn)) {
    info.serial_number = "Unknown";
    info.device = DeviceType::LidarUnknown;
  } else {
    std::string ml30b1_sn_prefix = "1000";
    std::string ml30sa1_sn_prefix = "1001";
    std::string mlx_sn_prefix = "1002";
    const int hrd_version_pos = 5;
    const int hrd_version_len = 1;

    if (0 == sn.compare(0, ml30b1_sn_prefix.size(), ml30b1_sn_prefix)) // ML30B1
    {
      info.device = DeviceType::LidarML30B1;
    } else if (0 == sn.compare(0, ml30sa1_sn_prefix.size(),
                               ml30sa1_sn_prefix)) // ML30SA1
    {
      if (0 == sn.compare(hrd_version_pos, hrd_version_len, "1"))
        info.device = DeviceType::LidarML30SA1;
      else if (0 == sn.compare(hrd_version_pos, hrd_version_len, "2"))
        info.device = DeviceType::LidarML30SB1;
      else if (0 == sn.compare(hrd_version_pos, hrd_version_len, "3"))
        info.device = DeviceType::LidarML30SB2;
      else
        info.device = DeviceType::LidarUnknown;
    } else if (0 == sn.compare(0, mlx_sn_prefix.size(), mlx_sn_prefix)) // MLX
    {
      info.device = DeviceType::LidarMLX;
    }

    info.serial_number = sn;
  }

  // backup firmware version
  memset(info.backup_version.boot_version, 0, 4);
  memset(info.backup_version.kernel_version, 0, 4);
  if (0 != QueryDeviceBackupFirmwareVersion(info.backup_version)) {
    LOG_F(ERROR, "Query backup firmware version error.");
  }

  // get device factory mac
  std::string fcmac = "Unknown";
  if (QueryDeviceFactoryMac(fcmac)) {
    info.factory_mac = "Unknown";
  } else {
    info.factory_mac = fcmac;
  }

  // update for dhcp [ip|mask|gateway|dstIp] ,trick ->
  std::string dhcpNetAddr;
  if (QueryDeviceHdcpNetAddr(dhcpNetAddr)) {
    info.dhcp_enable = StateMode::StateUnknown;
  } else {
    // update
    unsigned char *pdata = (unsigned char *)dhcpNetAddr.c_str();
    ResolveIpString(pdata, info.device_ip);
    ResolveIpString(pdata + 4, info.subnet_mask);
    ResolveIpString(pdata + 8, info.gateway_addr);
    ResolveIpString(pdata + 12, info.destination_ip);
  }

  // get device algo param
  QueryDeviceAlgoParam(info.algo_param);
  return 0;
}

int LidarTools::QueryDeviceAlgoParam(DeviceAlgoParam &param) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Read configuration info*/
  const int send_len = 4;
  char cfg_read_cmd[send_len] = {(char)0xBA, (char)0x0B, (char)0x01,
                                 (char)0x00};
  std::string cmd(cfg_read_cmd, 4);

  const int recv_len = 104;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, 4)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  // check whether algo param is valid
  int len = client_->GetAvailableBytesLen();
  if (len != 100) {
    std::string temp(len, 'x');
    client_->SyncRecv(temp, len);
    return TcpRecvTimeout;
  }

  if (client_->SyncRecv(recv, 100)) // recv configuration data
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  unsigned char *header = (unsigned char *)recv.c_str();
  param.isValid = true;

  NetworkToHost(header + 0, (char *)&param.retro_dis_thres);
  NetworkToHostShort(header + 4, (char *)&param.retro_low_range_thres);
  NetworkToHostShort(header + 6, (char *)&param.retro_high_range_thres);

  NetworkToHost(header + 8, (char *)&param.adhesion_angle_hor_min);
  NetworkToHost(header + 12, (char *)&param.adhesion_angle_hor_max);
  NetworkToHost(header + 16, (char *)&param.adhesion_angle_ver_min);
  NetworkToHost(header + 20, (char *)&param.adhesion_angle_ver_max);
  NetworkToHost(header + 24, (char *)&param.adhesion_angle_hor_res);
  NetworkToHost(header + 28, (char *)&param.adhesion_angle_ver_res);
  NetworkToHost(header + 32, (char *)&param.adhesion_diff_thres);
  NetworkToHost(header + 36, (char *)&param.adhesion_dis_limit);

  param.retro_min_gray_num = header[40];
  param.retro_del_gray_thres = header[41];
  param.retro_del_ratio_gray_low_thres = header[42];
  param.retro_del_ratio_gray_high_thres = header[43];
  param.retro_min_gray = header[44];

  NetworkToHost(header + 46, (char *)&param.adhesion_min_diff);

  return 0;
}

int LidarTools::GetDeviceCalibrationData(CalibrationData &cal) {
  int ret = 0;
  CalibrationPackets pkts;
  ret = LidarTools::GetDeviceCalibrationPackets(pkts);
  if (ret)
    return ret;
  ret = LidarTools::GetDeviceCalibrationData(pkts, cal);
  if (ret)
    return ret;

  return 0;
}

int LidarTools::GetDeviceCalibrationPackets(CalibrationPackets &pkts,
                                            std::string cmd) {
  const int ppf = 256000; // points per frame, 256000 reserved
  const int ppk = 128;    // points per cal udp packet
  std::unique_ptr<float> angle_data(
      new float[ppf * 2]); // points( azimuh, elevation);
  int packet_buffer_size =
      1040 * (ppf / ppk) +
      4; // 128 points in one packet, buffer reserved for ppf points.
  std::unique_ptr<unsigned char> packet_data(
      new unsigned char[packet_buffer_size]);
  const int send_len = 4;
  char cal_cmd[send_len] = {(char)0xBA, (char)0x07, (char)0x00, (char)0x00};
  std::string str_cmd(cal_cmd, send_len);

  // use user input tcp cmd
  if (cmd.size() == 4)
    str_cmd = cmd;

  pkts.clear();

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (!CheckConnection())
    return TcpConnTimeout;

  if (client_->SyncSend(str_cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  const int cal_pkt_len = 1040;
  std::string cal_recv(cal_pkt_len, 'x');
  // CalibrationPacket* pkt = reinterpret_cast<CalibrationPacket*>((char
  // *)cal_recv.c_str());

  // receive first packet to identify the device type
  if (client_->SyncRecv(cal_recv, cal_pkt_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  int total_packet = 0;
  ScanMode sm = CalibrationPacket::GetScanMode(cal_recv);

  if (ScanMode::ScanML30B1_100 == sm) {
    total_packet = 235; // 10000 * 3 * 2 * 4 / 1024
  } else if (ScanMode::ScanML30SA1_160 == sm ||
             ScanMode::ScanML30SA1Plus_160 == sm) {
    total_packet = 400; // 6400 * 8 * 2 * 4 / 1024
  } else if (ScanMode::ScanML30SA1_160_1_2 == sm ||
             ScanMode::ScanML30SA1Plus_160_1_2 == sm) {
    total_packet = 200; // 6400 * 8 * 2 * 4 / 1024 / 2
  } else if (ScanMode::ScanML30SA1_160_1_4 == sm ||
             ScanMode::ScanML30SA1Plus_160_1_4 == sm) {
    total_packet = 100; // 6400 * 8 * 2 * 4 / 1024 / 4
  } else if (ScanMode::ScanML30SA1_190 == sm) {
    total_packet = 450; // 7200 * 8 * 2 * 4 / 1024
  } else if (ScanMode::ScanMLX_160 == sm) {
    total_packet = 750; // 32000 *3 * 2 * 4 / 1024
  } else if (ScanMode::ScanMLXS_180 == sm) {
    total_packet = 844; // 36000 *3 * 2 * 4 / 1024
  } else {
    DisConnect();
    return NotSupport; // not support
  }

  pkts.push_back(cal_recv);
  for (int i = 0; i < total_packet - 1; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(110));
    int ret = client_->SyncRecv(cal_recv, cal_pkt_len);
    if (ret) {
      LOG_F(ERROR, "Receive calibration data error, ret = %d.", ret);
      DisConnect();
      return TcpRecvTimeout;
    }
    pkts.push_back(cal_recv);
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  if (total_packet != pkts.size())
    return NotEnoughData;

  return 0;
}

int LidarTools::GetDeviceCalibrationData(const CalibrationPackets &pkts,
                                         CalibrationData &cal) {
  if (!pkts.size())
    return InvalidContent;

  CalibrationPackets &packets = const_cast<CalibrationPackets &>(pkts);
  ScanMode sm = CalibrationPacket::GetScanMode(packets[0]);

  // v0.0.7 compatible
  int data_offset = 16;
  if (ScanMode::ScanUnknown == sm) {
    data_offset = 5;
    sm = ScanMode::ScanML30SA1_160;
  }

  for (int i = 0; i < pkts.size(); i++) {
    char *cal_data_ = (char *)packets[i].c_str();
    int network_data = 0;
    int host_data = 0;
    float *pfloat_data = reinterpret_cast<float *>(&host_data);
    for (int j = 0; j < 128 * 2; j++) {
      memcpy(&network_data, cal_data_ + j * 4 + data_offset,
             4); // 4 bytes per data, azimuth, elevation, 16 bytes header
      host_data = ntohl(network_data);
      cal.data.push_back(*pfloat_data);
    }
  }

  if (ScanMode::ScanML30B1_100 == sm) {
    cal.data.resize(
        10000 * 3 *
        2); // 10000 for per sub fov, 3 fovs, 2(azimuth and elevation)
  }
  else if (ScanMode::ScanML30SA1_160 == sm || ScanML30SA1Plus_160 == sm || ScanML30SA1Plus_160 == sm) {
    cal.data.resize(6400 * 8 * 2);
  } else if (ScanMode::ScanML30SA1_160_1_2 == sm ||
             ScanML30SA1Plus_160_1_2 == sm) {
    cal.data.resize(6400 * 8 * 2 / 2);
  } else if (ScanMode::ScanML30SA1_160_1_4 == sm ||
             ScanML30SA1Plus_160_1_4 == sm) {
    cal.data.resize(6400 * 8 * 2 / 4);
  } else if (ScanMode::ScanML30SA1_190 == sm) {
    cal.data.resize(6400 * 8 * 2);
  } else if (ScanMode::ScanMLX_160 == sm) {
    cal.data.resize(32000 * 3 * 2);
  } else if (ScanMode::ScanMLXS_180 == sm) {
    cal.data.resize(36000 * 3 * 2);
  } else {
  }
 
  if (!zvision::is_ml30splus_b1_ep_mode_enable()) {

    // now we reorder the cali data   0~15... -> 0~7... + 8~15...
    if (ScanML30SA1Plus_160 == sm || ScanML30SA1Plus_160_1_2 == sm ||
        ScanML30SA1Plus_160_1_4 == sm) {
      std::vector<float> dst;
      dst.resize(cal.data.size(), .0f);
      int id_h = 0;
      int id_l = cal.data.size() / 2;
      for (int i = 0; i < cal.data.size(); i++) {
        if (i / 8 % 2 == 0) {
          dst.at(id_h) = cal.data[i];
          id_h++;
        } else {
          dst.at(id_l) = cal.data[i];
          id_l++;
        }
      }
      cal.data = dst;
    }
  }

  cal.scan_mode = sm;
  return 0;
}

int LidarTools::GetDeviceCalibrationDataToFile(std::string filename,
                                               zvision::DeviceType tp) {
  CalibrationData cal;
  int ret = 0;
  if (tp == zvision::DeviceType::LidarMl30SA1Plus) {
    CalibrationPackets pkts;
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x0A, (char)0x00};
    ret = LidarTools::GetDeviceCalibrationPackets(pkts, std::string(cmd, 4));
    if (ret)
      return ret;
    ret = LidarTools::GetDeviceCalibrationData(pkts, cal);
  } else {
    ret = GetDeviceCalibrationData(cal);
  }

  if (ret)
    return ret;
  else
    return LidarTools::ExportCalibrationData(cal, filename);
}

int LidarTools::SetDeviceCalibrationData(std::string filename,
                                         zvision::DeviceType tp) {

  // 1. read cali data from file
  CalibrationData cal;
  int ret = 0;
  if (0 != (ret = LidarTools::ReadCalibrationData(filename, cal)))
    return ret;

  // only support for ml30sa1
  if (cal.scan_mode != ScanML30SA1_160 && cal.scan_mode != ScanML30SA1Plus_160)
    return NotSupport;

  if (!zvision::is_ml30splus_b1_ep_mode_enable()) {
    // we need trans cali data
    if (cal.scan_mode == ScanML30SA1Plus_160) {
      int lpos = cal.data.size() / 2;
      std::vector<float> cal_tmp;
      std::vector<float> cal_fov0_3 =
          std::vector<float>{std::begin(cal.data), std::begin(cal.data) + lpos};
      std::vector<float> cal_fov4_7 = std::vector<float>{
          std::begin(cal.data) + lpos, std::begin(cal.data) + cal.data.size()};
      int groups = cal.data.size() / 16;
      for (int g = 0; g < groups; g++) {
        float val = .0f;
        for (int col = 0; col < 16; col++) {
          int id = g * 16 + col;
          if (col < 8) {
            val = cal_fov0_3.at(g * 8 + col);
          } else {
            val = cal_fov4_7.at(g * 8 + col % 8);
          }
          cal.data.at(id) = val;
        }
      }
    }
  }

  const int cali_pkt_len = 1024;
  size_t cali_data_len = cal.data.size();
  // 2. generate calibration buffer
  std::vector<std::string> cali_pkts;
  std::string pkt(cali_pkt_len, '0');
  for (int i = 0; i < cali_data_len; i++) {
    // float to str
    char *pdata = (char *)(&cal.data[i]);
    int *paddr = (int *)pdata;
    *paddr = ntohl(*paddr);
    // update packet
    for (int j = 0; j < 4; j++)
      pkt.at(i * 4 % cali_pkt_len + j) = *(pdata + j);

    // generate packet
    if ((i + 1) % 256 == 0) {
      cali_pkts.push_back(pkt);
      pkt = std::string(cali_pkt_len, '0');
    }
  }

  if (!CheckConnection())
    return TcpConnTimeout;

  // 3. send cmd
  const int cmd_len = 4;
  char cal_cmd[cmd_len] = {(char)0xAB, (char)0x04, (char)0x00, (char)0x00};
  std::string cmd(cal_cmd, cmd_len);
  if (client_->SyncSend(cmd, cmd_len)) {
    DisConnect();
    return TcpSendTimeout;
  }
  // recv ret
  std::string recv(cmd_len, 'x');
  if (client_->SyncRecv(recv, cmd_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }
  // check ret
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  // 4. send cali data to lidar
  for (int i = 0; i < cali_pkts.size(); i++) {
    ret = client_->SyncSend(cali_pkts[i], cali_pkts[i].size());
    if (ret) {
      LOG_F(ERROR, "Send calibration data error, ret = %d.", ret);
      DisConnect();
      return TcpSendTimeout;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(110));
  }

  // 5. check
  recv = std::string(cmd_len, 'x');
  if (client_->SyncRecv(recv, cmd_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }
  // check ret
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceStaticIpAddress(std::string ip) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device static ip address*/
  const int send_len = 6;
  char set_cmd[send_len] = {(char)0xBA, (char)0x03, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  if (!AssembleIpString(ip, set_cmd + 2))
    return InvalidParameter;

  std::string bv_cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(bv_cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::ResetDeviceMacAddress() {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Reset device mac address */
  const int send_len = 9;
  char set_cmd[send_len] = {(char)0xBA, (char)0x0A, (char)0xFF, (char)0xFF,
                            (char)0xFF, (char)0xFF, (char)0xFF, (char)0xFF};

  std::string bv_cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(bv_cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceSubnetMask(std::string mask) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device subnet mask, default 255.255.255.0*/
  const int send_len = 6;
  char set_cmd[send_len] = {(char)0xBA, (char)0x09, (char)0xFF,
                            (char)0xFF, (char)0xFF, (char)0x00};

  if (!AssembleIpString(mask, set_cmd + 2))
    return InvalidParameter;

  std::string bv_cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(bv_cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceMacAddress(std::string mac) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device subnet mask, default FF:FF:FF:FF:FF:FF*/
  const int send_len = 8;
  char set_cmd[send_len] = {(char)0xBA, (char)0x0A, (char)0xFF, (char)0xFF,
                            (char)0xFF, (char)0xFF, (char)0xFF, (char)0xFF};

  if (!AssembleMacAddress(mac, set_cmd + 2))
    return InvalidParameter;

  std::string bv_cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(bv_cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceUdpDestinationIpAddress(std::string ip) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device udp destination ip address*/
  const int send_len = 6;
  char set_cmd[send_len] = {(char)0xBA, (char)0x06, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  if (!AssembleIpString(ip, set_cmd + 2))
    return InvalidParameter;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceUdpDestinationPort(int port) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device udp destination port*/
  const int send_len = 6;
  char set_cmd[send_len] = {(char)0xBA, (char)0x04, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  AssemblePort(port, set_cmd + 2);

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceTimestampType(TimestampType tp) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device timestamp type*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x05, (char)0x00, (char)0x00};

  if (TimestampPtp == tp)
    set_cmd[2] = 0x01;
  else if (TimestampPpsGps == tp)
    set_cmd[2] = 0x02;
  else
    return NotSupport;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceAlgorithmEnable(AlgoType tp, bool en) {

  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device Algorithm enable*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x1F, (char)0x00, (char)0x00};

  if (AlgoDeleteClosePoints == tp)
    set_cmd[2] = 0x01;
  else if (AlgoAdhesion == tp)
    set_cmd[2] = 0x02;
  else if (AlgoRetro == tp)
    set_cmd[2] = 0x03;
  else
    return NotSupport;

  if (en)
    set_cmd[3] = 0x01;
  else
    set_cmd[3] = 0x00;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceRetroEnable(bool en) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device timestamp type*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xAB, (char)0x03, (char)0x00, (char)0x00};

  if (en)
    set_cmd[2] = 0x01;
  else
    set_cmd[2] = 0x00;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceRetroParam1MinRef(int ref_min) {
  if ((ref_min < 0) || (ref_min > 250))
    return InvalidParameter;

  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device retro param 1*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x15, (char)0x01, (char)0x00};

  set_cmd[3] = ref_min & 0xFF;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceRetroParam2PointPercentage(int percentage) {
  if ((percentage < 0) || (percentage > 100))
    return InvalidParameter;

  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device retro param 2*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x15, (char)0x02, (char)0x00};

  set_cmd[3] = percentage & 0xFF;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceAdhesionParam(AdhesionParam tp, int val) {

  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 7;
  char set_cmd[send_len] = {(char)0xBA, (char)0x20, (char)0x00, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  // check param
  if (MinimumHorizontalAngleRange == tp)
    set_cmd[2] = 0x01;
  else if (MaximumHorizontalAngleRange == tp)
    set_cmd[2] = 0x02;
  else if (MinimumVerticalAngleRange == tp)
    set_cmd[2] = 0x03;
  else if (MaximumVerticalAngleRange == tp)
    set_cmd[2] = 0x04;
  else
    return NotSupport;

  HostToNetwork((const unsigned char *)&val, set_cmd + 3);

  std::string cmd(set_cmd, send_len);
  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceAdhesionParam(AdhesionParam tp, float val) {

  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 7;
  char set_cmd[send_len] = {(char)0xBA, (char)0x20, (char)0x00, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  // check param
  if (HorizontalAngleResolution == tp)
    set_cmd[2] = 0x05;
  else if (VerticalAngleResolution == tp)
    set_cmd[2] = 0x06;
  else if (DeletePointThreshold == tp)
    set_cmd[2] = 0x07;
  else if (MaximumProcessingRange == tp)
    set_cmd[2] = 0x08;
  else if (NearFarPointDiff == tp)
    set_cmd[2] = 0x09;
  else
    return NotSupport;

  HostToNetwork((const unsigned char *)&val, set_cmd + 3);

  std::string cmd(set_cmd, send_len);
  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceRetroParam(RetroParam tp, int val) {

  if (!CheckConnection())
    return TcpConnTimeout;

  if (tp != RetroParam::RetroDisThres)
    return NotSupport;

  const int send_len = 7;
  char set_cmd[send_len] = {(char)0xBA, (char)0x15, (char)0x02, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  HostToNetwork((const unsigned char *)&val, set_cmd + 3);
  std::string cmd(set_cmd, send_len);

  // send
  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  // recv
  const int recv_len = 4;
  std::string recv(recv_len, 'x');
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // check
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceRetroParam(RetroParam tp, unsigned short val) {

  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 5;
  char set_cmd[send_len] = {(char)0xBA, (char)0x15, (char)0x00, (char)0x00,
                            (char)0x00};

  if (tp == RetroParam::RetroLowRangeThres)
    set_cmd[2] = 0x03;
  else if (tp == RetroParam::RetroHighRangeThres)
    set_cmd[2] = 0x04;
  else
    return NotSupport;

  set_cmd[3] = val >> 8 & 0xFF;
  set_cmd[4] = val & 0xFF;

  std::string cmd(set_cmd, send_len);

  // send
  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  // recv
  const int recv_len = 4;
  std::string recv(recv_len, 'x');
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // check
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }
  return 0;
}

int LidarTools::SetDeviceRetroParam(RetroParam tp, unsigned char val) {

  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x15, (char)0x00, (char)0x00};

  if (tp == RetroParam::RetroMinGrayNum)
    set_cmd[2] = 0x01;
  else if (tp == RetroParam::RetroDelGrayThres)
    set_cmd[2] = 0x05;
  else if (tp == RetroParam::RetroDelRatioGrayLowThres)
    set_cmd[2] = 0x06;
  else if (tp == RetroParam::RetroDelRatioGrayHighThres)
    set_cmd[2] = 0x07;
  else if (tp == RetroParam::RetroMinGray)
    set_cmd[2] = 0x08;
  else
    return NotSupport;

  set_cmd[3] = val;
  std::string cmd(set_cmd, send_len);

  // send
  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  // recv
  const int recv_len = 4;
  std::string recv(recv_len, 'x');
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // check
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::FirmwareUpdate(std::string &filename, ProgressCallback cb) {
  if (!CheckConnection())
    return TcpConnTimeout;

  std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
  if (!in.is_open()) {
    LOG_F(ERROR, "Open file error.");
    return OpenFileError;
  }

  std::streampos end = in.tellg();
  int size = static_cast<int>(end);
  in.close();

  const int send_len = 6;
  char upgrade_cmd[send_len] = {(char)0xBA, (char)0x01, (char)0x00,
                                (char)0x00, (char)0x00, (char)0x00};
  HostToNetwork((const unsigned char *)&size, upgrade_cmd + 2);
  std::string cmd(upgrade_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  // transfer, erase flash, write
  const int pkt_len = 256;
  int start_percent = 10;
  int block_total = size / pkt_len;
  if (0 != (size % pkt_len))
    block_total += 1;
  int step_per_percent = block_total / 30; // old is 90

  std::ifstream idata(filename, std::ios::in | std::ios::binary);
  char fw_data[pkt_len] = {0x00};
  int readed = 0;

  // data transfer
  if (idata.is_open()) {
    for (readed = 0; readed < block_total; readed++) {
      int read_len = pkt_len;
      if ((readed == (block_total - 1)) && (0 != (size % pkt_len)))
        read_len = size % pkt_len;
      idata.read(fw_data, read_len);
      std::string fw(fw_data, read_len);
      if (client_->SyncSend(fw, read_len)) {
        break;
      }
      if (0 == (readed % step_per_percent)) {
        cb(start_percent++, this->device_ip_.c_str());
      }
    }
    idata.close();
  }
  if (readed != block_total) {
    client_->Close();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }
  LOG_F(2, "Data transfer ok...");

  // waitting for step 2
  const int recv_step_len = 5;
  std::string recv_step(recv_step_len, 'x');
  bool ok = false;
  while (1) {
    if (client_->SyncRecv(recv_step, recv_step_len)) {
      ok = false;
      break;
    }
    unsigned char step = (unsigned char)recv_step[4];
    cb(40 + int((double)step / 3.3), this->device_ip_.c_str());
    if (100 == step) {
      ok = true;
      break;
    }
  }

  if (!ok) {
    LOG_F(ERROR, "Waiting for erase flash failed...");
    DisConnect();
    return TcpRecvTimeout;
  }
  LOG_F(2, "Waiting for erase flash ok...");

  // waitting for step 3
  while (1) {
    if (client_->SyncRecv(recv_step, recv_step_len)) {
      ok = false;
      break;
    }
    unsigned char step = (unsigned char)recv_step[4];
    cb(70 + int((double)step / 3.3), this->device_ip_.c_str());
    if (100 == step) {
      ok = true;
      break;
    }
  }

  if (!ok) {
    LOG_F(ERROR, "Waiting for write flash failed...");
    DisConnect();
    return TcpRecvTimeout;
  }
  LOG_F(2, "Waiting for write flash ok...");

  // device check data
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::RebootDevice() {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device timestamp type*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x0C, (char)0x00, (char)0x00};

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::GetDeviceLog(std::string &log) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Get device log info*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x0D, (char)0x00, (char)0x00};

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  // get log buffer length
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }
  uint32_t log_buffer_len = 0;
  NetworkToHost((const unsigned char *)recv.c_str(), (char *)&log_buffer_len);

  // get log buffer
  std::string log_buffer;
  log_buffer.resize(log_buffer_len);
  if (client_->SyncRecv(log_buffer, log_buffer_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // assemble the buffer to string
  unsigned char file_count = reinterpret_cast<unsigned char &>(log_buffer[0]);

  std::vector<std::string> file_contents;
  uint32_t position = 1;
  for (int i = 0; i < file_count; ++i) {
    uint32_t file_len = 0;
    NetworkToHost((const unsigned char *)(&log_buffer[position]),
                  (char *)&file_len);
    LOG_F(INFO, "FILE %d len %u.", i, file_len);
    std::string content(log_buffer, position + 4,
                        file_len); // copy to file list
    file_contents.push_back(content);
    position += (file_len + 4);
  }

  // reorder the log file
  log = "";
  for (int i = file_count - 1; i >= 0; i--) {
    log += file_contents[i];
  }

  // final ret
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDevicePhaseOffset(uint32_t offset) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Phase offset*/
  const int send_len = 6;
  char set_cmd[send_len] = {(char)0xBA, (char)0x0E, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  HostToNetwork((const unsigned char *)&offset, set_cmd + 2);

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDevicePhaseOffsetEnable(bool en) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device phase offset enable*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x14, (char)0x00, (char)0x00};
  if (en)
    set_cmd[2] = 0x01;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDevicePtpConfiguration(std::string ptp_cfg_filename) {
  //// --- read ptp configuration file content
  // read file
  std::string data;
  char c;
  std::ifstream inFile(ptp_cfg_filename, std::ios::in | std::ios::binary);
  if (!inFile)
    return ReturnCode::OpenFileError;

  while ((inFile.get(c))) {
    if (inFile.eof())
      break;
    data.push_back(c);
  }
  inFile.close();

  int length = data.size();
  std::string content = data;

  if (!CheckConnection())
    return TcpConnTimeout;

  // --- send data to device
  const int send_len = 6;
  char set_cmd[send_len] = {(char)0xBA, (char)0x0F, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  uint32_t file_len = length;
  HostToNetwork((const unsigned char *)&file_len, set_cmd + 2);

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  // send file to device
  if (client_->SyncSend(content, file_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  // final ack
  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return ReturnCode::Success;
}

int LidarTools::SetDeviceConfigFile(std::string cfg_filename) {

  // read file data
  std::string data;
  std::ifstream inFile(cfg_filename, std::ios::in | std::ios::binary);
  if (!inFile)
    return ReturnCode::OpenFileError;

  std::stringstream ss;
  ss << inFile.rdbuf();
  data = ss.str();
  inFile.close();

  int length = data.size();
  std::string content = data;
  if (!CheckConnection())
    return TcpConnTimeout;

  // --- send data to device
  const int send_len = 7;
  char set_cmd[send_len] = {(char)0xBA, (char)0x21, (char)0x01, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  uint32_t file_len = length;
  HostToNetwork((const unsigned char *)&file_len, set_cmd + 3);

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  // send file to device
  if (client_->SyncSend(content, file_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  // final ack
  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return ReturnCode::Success;
  return 0;
}

int LidarTools::QueryDeviceConfigFileVersion(std::string &ver) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Read version info*/
  const int send_len = 4;
  char bv_read_cmd[send_len] = {(char)0xBA, (char)0x02, (char)0x03, (char)0x00};
  std::string bv_cmd(bv_read_cmd, 4);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(bv_cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  const int recv_ver_len = 3;
  std::string recv_ver(recv_ver_len, 'x');
  if (client_->SyncRecv(recv_ver, recv_ver_len)) // recv version data
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  char cver[128] = "";
  const uint8_t *pver = (uint8_t *)recv_ver.data();
  sprintf_s(cver, "%u.%u.%u", *(pver + 0), *(pver + 1), *(pver + 2));
  ver = std::string(cver);

  return 0;
}

int LidarTools::GetDevicePtpConfiguration(std::string &ptp_cfg) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Get device log info*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x10, (char)0x00, (char)0x00};

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  // get ptp configuration data
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }
  uint32_t ptp_buffer_len = 0;
  NetworkToHost((const unsigned char *)recv.c_str(), (char *)&ptp_buffer_len);

  // get ptp configuration buffer
  std::string ptp_cfg_buffer(ptp_buffer_len, 'x');
  if (client_->SyncRecv(ptp_cfg_buffer, ptp_buffer_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  ptp_cfg = ptp_cfg_buffer;

  return 0;
}

int LidarTools::GetDevicePtpConfigurationToFile(std::string &save_file_name) {
  std::string content = "";

  int ret = GetDevicePtpConfiguration(content);
  if (ret)
    return ret;
  else {
    std::ofstream out(save_file_name, std::ios::out | std::ios::binary);
    if (out.is_open()) {
      out.write(content.c_str(), content.size());
      out.close();
      return 0;
    } else
      return OpenFileError;
  }
}

int LidarTools::SetDevicePointFireEnConfiguration(std::string fire_en_filename,
                                                  DeviceType devType) {
  // --- read point fire enbale configuration file content
  std::ifstream infile(fire_en_filename);
  if (!infile.is_open()) {
    return ReturnCode::OpenFileError;
  }

  std::string line;
  std::vector<int> ids;
  const int column = 1;
  while (std::getline(infile, line)) {
    int value = 0xFFFFFFFF;
    if (line.size() > 0) {
      int match = sscanf_s(line.c_str(), "%x", (unsigned int *)&value);

      // printf("%08X\n", value);
      if (column != match)
        break;
      ids.push_back(value);
    }
  }
  infile.close();

  if (ids.size() != 1600)
    return InvalidContent;

  if (!CheckConnection())
    return TcpConnTimeout;

  // --- send data to device
  const int send_len = 6;
  char set_cmd[send_len] = {(char)0xAB, (char)0x01, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  uint32_t size_len = 2;
  uint32_t data_len = 1600 * 4 + size_len;
  HostToNetwork((const unsigned char *)&data_len, set_cmd + 2);
  std::string cmd(set_cmd, send_len);

  std::string content;
  content.resize(data_len);
  if (devType == DeviceType::LidarML30B1 ||
      devType == DeviceType::LidarML30SB1) {
    content[0] = 0x02; // 02->00
    content[1] = 0x00; // 00->40
  } else {
    content[0] = 0x00; // 02->00
    content[1] = 0x40; // 00->40
  }
  // content[0] = 0x00; // 02->00
  // content[1] = 0x40; // 00->40
  std::unique_ptr<int> point_fire_en_data(new int[ids.size()]);
  int *point_fire_en_data_ptr = point_fire_en_data.get();

  for (int i = 0; i < ids.size(); i++) {
    point_fire_en_data_ptr[i] = ids[i];
  }
  char *send = (char *)content.c_str() + 2;
  HostToNetwork((const unsigned char *)point_fire_en_data_ptr, send, 1600 * 4);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');
  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  // send file to device
  if (client_->SyncSend(content, data_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  // final ack
  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return ReturnCode::Success;
}

int LidarTools::BackupFirmwareUpdate(std::string &filename,
                                     ProgressCallback cb) {
  if (!CheckConnection())
    return TcpConnTimeout;

  std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
  if (!in.is_open()) {
    LOG_F(ERROR, "Open file error.");
    return OpenFileError;
  }

  std::streampos end = in.tellg();
  int size = static_cast<int>(end);
  in.close();

  const int send_len = 6;
  char upgrade_cmd[send_len] = {(char)0xBA, (char)0x11, (char)0x00,
                                (char)0x00, (char)0x00, (char)0x00};
  HostToNetwork((const unsigned char *)&size, upgrade_cmd + 2);
  std::string cmd(upgrade_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  // transfer, erase flash, write
  int start_percent = 10;
  const int pkt_len = 256;
  int block_total = size / pkt_len;
  if (0 != (size % pkt_len))
    block_total += 1;

  int step_per_percent = block_total / 30; // old is 90

  std::ifstream idata(filename, std::ios::in | std::ios::binary);
  char fw_data[pkt_len] = {0x00};
  int readed = 0;

  // data transfer
  if (idata.is_open()) {
    for (readed = 0; readed < block_total; readed++) {
      int read_len = pkt_len;
      if ((readed == (block_total - 1)) && (0 != (size % pkt_len)))
        read_len = size % pkt_len;
      idata.read(fw_data, read_len);
      std::string fw(fw_data, read_len);
      if (client_->SyncSend(fw, read_len)) {
        break;
      }
      if (0 == (readed % step_per_percent)) {
        cb(start_percent++, this->device_ip_.c_str());
      }
    }
    idata.close();
  }
  if (readed != block_total) {
    client_->Close();
    return TcpSendTimeout;
  }
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }
  LOG_F(2, "Data transfer ok...");

  // waitting for step 2
  const int recv_step_len = 5;
  std::string recv_step(recv_step_len, 'x');
  bool ok = false;
  while (1) {
    if (client_->SyncRecv(recv_step, recv_step_len)) {
      ok = false;
      LOG_F(ERROR, "Waiting for erase flash failed...");
      break;
    }
    unsigned char step = (unsigned char)recv_step[4];
    cb(40 + int((double)step / 3.3), this->device_ip_.c_str());
    if (100 == step) {
      ok = true;
      break;
    }
  }

  if (!ok) {
    DisConnect();
    return TcpRecvTimeout;
  }
  LOG_F(2, "Waiting for erase flash ok...");

  // waitting for step 3
  while (1) {
    if (client_->SyncRecv(recv_step, recv_step_len)) {
      ok = false;
      break;
    }
    unsigned char step = (unsigned char)recv_step[4];
    cb(70 + int((double)step / 3.3), this->device_ip_.c_str());
    if (100 == step) {
      ok = true;
      break;
    }
  }

  if (!ok) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // device check data
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::QueryDeviceBackupFirmwareVersion(FirmwareVersion &version) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Read version info*/
  const int send_len = 4;
  char bv_read_cmd[send_len] = {(char)0xBA, (char)0x12, (char)0x01, (char)0x00};
  std::string bv_cmd(bv_read_cmd, 4);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(bv_cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv version data
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  memcpy(&(version.boot_version), recv.c_str(), 4);

  char kv_read_cmd[send_len] = {(char)0xBA, (char)0x12, (char)0x02, (char)0x00};
  std::string kv_cmd(kv_read_cmd, 4);

  if (client_->SyncSend(kv_cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv version data
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  memcpy(&(version.kernel_version), recv.c_str(), 4);

  return 0;
}

int LidarTools::SetDeviceEchoMode(EchoMode mode) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device echo mode*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x13, (char)0x00, (char)0x00};

  if (EchoSingleFirst == mode)
    set_cmd[2] = 0x01;
  else if (EchoSingleStrongest == mode)
    set_cmd[2] = 0x02;
  else if (EchoSingleLast == mode)
    set_cmd[2] = 0x04;
  else if (EchoDoubleFirstStrongest == mode)
    set_cmd[2] = 0x03;
  else if (EchoDoubleFirstLast == mode)
    set_cmd[2] = 0x05;
  else if (EchoDoubleStrongestLast == mode)
    set_cmd[2] = 0x06;
  else
    return ReturnCode::InvalidParameter;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();

    // check
    if ((uint8_t)recv[2] == 0xFF && (uint8_t)recv[3] == 0x21) {
      return NotSupport;
    }
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceCalSendMode(CalSendMode mode) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device cal send mode*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x16, (char)0x00, (char)0x00};

  if (CalSendDisable == mode)
    set_cmd[2] = 0x00;
  else if (CalSendEnable == mode)
    set_cmd[2] = 0x01;
  else
    return ReturnCode::InvalidParameter;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceDownsampleMode(DownsampleMode mode) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device downsample*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x17, (char)0x00, (char)0x00};

  if (DownsampleNone == mode)
    set_cmd[2] = 0x00;
  else if (Downsample_1_2 == mode)
    set_cmd[2] = 0x01;
  else if (Downsample_1_4 == mode)
    set_cmd[2] = 0x02;
  else
    return ReturnCode::InvalidParameter;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceDHCPMode(StateMode mode) {

  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device dhcp mode*/
  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x1D, (char)0x00, (char)0x00};

  if (StateDisable == mode)
    set_cmd[2] = 0x00;
  else if (StateEnable == mode)
    set_cmd[2] = 0x01;
  else
    return ReturnCode::InvalidParameter;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceGatewayAddr(std::string addr) {

  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device gateway address*/
  const int send_len = 6;
  char set_cmd[send_len] = {(char)0xBA, (char)0x1E, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  if (!AssembleIpString(addr, set_cmd + 2))
    return InvalidParameter;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::SetDeviceFactoryMac(std::string &mac) {

  if (!CheckConnection())
    return TcpConnTimeout;

  /*Set device factory mac*/
  const int send_len = 8;
  char set_cmd[send_len] = {(char)0xAB, (char)0x07, (char)0x00, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00, (char)0x00};

  if (!AssembleFactoryMacAddress(mac, set_cmd + 2))
    return InvalidParameter;

  if (set_cmd[2] != (char)0xF8 || set_cmd[3] != (char)0xA9 ||
      set_cmd[4] != (char)0x1F)
    return InvalidParameter;

  std::string cmd(set_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

int LidarTools::GetDeviceChannelDataToFile(std::string path, bool curr) {
  if (!CheckConnection())
    return TcpConnTimeout;

  // command
  const int cmd_len = 4;
  char set_cmd[cmd_len] = {(char)0xAB, (char)0x0C, (char)0x06, (char)0x00};
  if (!curr) {
    set_cmd[2] = (char)0x05;
  }
  // send
  std::string cmd(set_cmd, cmd_len);
  if (client_->SyncSend(cmd, cmd_len)) {
    DisConnect();
    return TcpSendTimeout;
  }
  // receive
  std::string recv(cmd_len, 'x');
  if (client_->SyncRecv(recv, cmd_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }
  // check ret
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }
  // get channel data
  const int recv_len = 102400;
  int pkt_len = 1024;
  int pkt_cnt = recv_len / pkt_len;
  std::string recv_data;
  for (int i = 0; i < pkt_cnt; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(110));
    std::string tmp(pkt_len, 0);
    if (client_->SyncRecv(tmp, pkt_len)) {
      DisConnect();
      return TcpRecvTimeout;
    }
    recv_data += tmp;
  }

  // open file
  FILE *pf = NULL;
  pf = fopen(path.c_str(), "w");
  if (!pf)
    return OpenFileError;

  // write header
  fprintf(pf,
          "MEMORY_INITIALIZATION_RADIX=16;\nMEMORY_INITIALIZATION_VECTOR = \n");
  // save to file
  uint8_t *pdata = (uint8_t *)recv_data.data();
  for (int i = 0; i < recv_data.size() / 4; i++) {
    int pos = i * 4;
    fprintf(pf, "%02x%02x%02x%02x,\n", (pdata[pos + 0] & 0xFF),
            (pdata[pos + 1] & 0xFF), (pdata[pos + 2] & 0xFF),
            (pdata[pos + 3] & 0xFF));
  }
  fclose(pf);
  pf = NULL;
  return 0;
}

int LidarTools::GetDeviceChannelListDataToFile(std::string path) {
  if (!CheckConnection())
    return TcpConnTimeout;

  // command
  const int cmd_len = 4;
  char set_cmd[cmd_len] = {(char)0xAB, (char)0x0C, (char)0x04, (char)0x00};
  // send
  std::string cmd(set_cmd, cmd_len);
  if (client_->SyncSend(cmd, cmd_len)) {
    DisConnect();
    return TcpSendTimeout;
  }
  // receive
  std::string recv(cmd_len, 'x');
  if (client_->SyncRecv(recv, cmd_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }
  // check ret
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }
  // get channel data
  const int recv_len = 51200;
  int pkt_len = 1024;
  int pkt_cnt = recv_len / pkt_len;
  std::string recv_data;
  for (int i = 0; i < pkt_cnt; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(110));
    std::string tmp(pkt_len, 0);
    if (client_->SyncRecv(tmp, pkt_len)) {
      DisConnect();
      return TcpRecvTimeout;
    }
    recv_data += tmp;
  }

  // open file
  FILE *pf = NULL;
  pf = fopen(path.c_str(), "w");
  if (!pf)
    return OpenFileError;

  // save to file
  uint8_t *pdata = (uint8_t *)recv_data.data();
  for (int i = 0; i < recv_data.size(); i++) {
    fprintf(pf, "%d\n", pdata[i] & 0xFF);
  }
  fclose(pf);
  pf = NULL;
  return 0;
}

int LidarTools::GetDeviceFlashConfiguration(std::string &buf,
                                            FlashParamType type) {

  if (!CheckConnection())
    return TcpConnTimeout;

  // --- send data to device
  const int send_len = 11;
  char set_cmd[send_len] = {(char)0xAB, (char)0x02, (char)0x03, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  uint32_t read_offset_arr[] = {
      0,           256 * 1024,  512 * 1024,  768 * 1024,
      1024 * 1024, 1280 * 1024, 1536 * 1024, 1792 * 1024,
      0,           0,           0,           0,
      0,           0,           0,           3840 * 1024};
  uint32_t read_size_arr[] = {32, 12000, 12000, 102400, 640, 3080, 6400, 16004,
                              0,  0,     0,     0,      0,   0,    0,    19200};

  uint32_t read_offset = read_offset_arr[type];
  uint32_t read_size = read_size_arr[type];
  HostToNetwork((const unsigned char *)&read_offset, set_cmd + 3);
  HostToNetwork((const unsigned char *)&read_size, set_cmd + 7);
  std::string cmd(set_cmd, send_len);

  // send cmd
  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  const int recv_check = 4;
  std::string recv(recv_check, 'x');
  // recv ret
  if (client_->SyncRecv(recv, recv_check)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // check ret
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  // recv flash data
  buf = std::string(read_size, 'x');
  if (client_->SyncRecv(buf, read_size)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // rect ret
  recv = std::string(recv_check, 'x');
  if (client_->SyncRecv(recv, recv_check)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // final check
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  return ReturnCode::Success;
}

int LidarTools::SetDeviceFlashConfiguration(const std::string &buf,
                                            FlashParamType tp,
                                            DeviceType devType) {

  if (!CheckConnection())
    return TcpConnTimeout;

  // 1. generate send command
  uint32_t read_size_arr[] = {32, 12000, 12000, 102400, 640, 3080, 6400, 16004,
                              0,  0,     0,     0,      0,   0,    0,    19200};
  const int send_len = 6;
  char set_cmd[send_len] = {(char)0xAB, (char)0x01, (char)0x00,
                            (char)0x00, (char)0x00, (char)0x00};

  uint32_t size_len = 2;
  uint32_t data_len = read_size_arr[tp] + size_len;
  HostToNetwork((const unsigned char *)&data_len, set_cmd + 2);
  std::string cmd(set_cmd, send_len);
  // check valid
  if ((data_len - 2) != buf.size())
    return InvalidParameter;

  // 2.1 generate flash data command type
  std::string content;
  content.resize(data_len);
  switch (tp) {
  case zvision::StartDelay1_3:
    content[0] = 0x00;
    content[1] = 0x01;
    break;
  case zvision::MEMS_X:
    content[0] = 0x00;
    content[1] = 0x02;
    break;
  case zvision::MEMS_Y:
    content[0] = 0x00;
    content[1] = 0x04;
    break;
  case zvision::APD_BestChannel:
    content[0] = 0x00;
    content[1] = 0x08;
    break;
  case zvision::APD_Delay:
    content[0] = 0x00;
    content[1] = 0x10;
    break;
  case zvision::ADC_Algo:
    content[0] = 0x00;
    content[1] = 0x20;
    break;
  case zvision::Cover:
    if (devType == DeviceType::LidarML30B1 ||
        devType == DeviceType::LidarML30SB1) {
      content[0] = 0x02;
      content[1] = 0x00;
    } else {
      content[0] = 0x00;
      content[1] = 0x40;
    }
    break;
  case zvision::DirtyCheck:
    content[0] = 0x00;
    content[1] = 0x80;
    break;
  default:
    break;
  }

  // 2.2 generate flash data
  char *send = (char *)content.c_str() + 2;
  memcpy(send, (char *)buf.c_str(), data_len - 2);

  // 3.1 send command
  const int recv_len = 4;
  std::string recv(recv_len, 'x');
  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  // 3.2 send flash data to device
  if (client_->SyncSend(content, data_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  return ReturnCode::Success;
}

int LidarTools::SetDeviceLayerDetectionEnable(bool en) {
  int ret = -1;
  if (!CheckConnection())
    return TcpConnTimeout;

  // 1. read hardware diagnostic control
  DeviceConfigurationInfo info;
  if (0 != (ret = QueryDeviceConfigurationInfo(info)))
    return ret;

  // 2. set layer detection switch
  const int send_len = 7;
  char set_cmd[send_len] = {(char)0xBA, (char)0x1B, (char)0x03, (char)0xFF,
                            (char)0xFF, (char)0xFF, (char)0xFF};
  if (en)
    info.hard_diag_ctrl |= 0x00080000;
  else
    info.hard_diag_ctrl &= 0xFFF7FFFF;
  HostToNetwork((const unsigned char *)&info.hard_diag_ctrl, set_cmd + 3);

  std::string cmd(set_cmd, send_len);
  const int recv_len = 4;
  std::string recv(recv_len, 'x');
  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }
  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }
  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }
  return 0;
}

int LidarTools::SetDeviceDeleteNearPointLevel(int mode) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 4;
  char set_cmd[send_len] = {(char)0xBA, (char)0x25, (char)0x00, (char)0x00};
  if (mode == 0)
    set_cmd[2] = 0x00;
  else if (mode == 1)
    set_cmd[2] = 0x01;
  else
    return InvalidParameter;

  std::string cmd(set_cmd, send_len);
  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) // send cmd
  {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) // recv ret
  {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) // check ret
  {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

bool LidarTools::CheckConnection() {
  if (!conn_ok_) {
    int ret = client_->Connect(this->device_ip_);
    if (ret) {
      client_->GetSysErrorCode();
      return false;
    }
    conn_ok_ = true;
  }

  return true;
}

void LidarTools::DisConnect() {
  if (conn_ok_) {
    this->client_->Close();
    conn_ok_ = false;
  }
}

bool LidarTools::CheckDeviceRet(std::string ret) {
  return (0x00 == ret[2]) && (0x00 == ret[3]);
}

/* for mlxs only */
int LidarTools::QueryMLXSDeviceNetworkConfigurationInfo(
    MLXSDeviceNetworkConfigurationInfo &info) {
  if (!CheckConnection())
    return TcpConnTimeout;

  /*Read configuration info*/
  const int send_len = 9;
  char mlxs_cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01,
                             (char)0x00, (char)0x02, (char)0x0D,
                             (char)0x03, (char)0x00, (char)0x00};
  uint16_t chk_sum = get_check_sum(std::string(mlxs_cmd, 7));
  mlxs_cmd[7] = (chk_sum >> 8) & 0xFF;
  mlxs_cmd[8] = chk_sum & 0xFF;
  std::string cmd(mlxs_cmd, 9);

  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  // recv header
  int header_len = 5;
  std::string header_recv(header_len, 'x');
  if (client_->SyncRecv(header_recv, header_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  // check ret header
  uint8_t *pheader = (uint8_t *)header_recv.data();
  if ((*(pheader + 0) != 0xBA) || (*(pheader + 1) != 0xAC) ||
      (*(pheader + 4) != 0x00)) {
    DisConnect();
    return DevAckError;
  }

  // get recv data
  uint16_t data_len = *(pheader + 2) << 8 | *(pheader + 3);
  std::string data_recv(data_len, 'x');
  if (client_->SyncRecv(data_recv, data_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (data_len >= 0x1B) {
    // parsing
    uint8_t *pdata = (uint8_t *)data_recv.data();
    info.sw_dhcp = *(pdata + 0);
    ResolveIpString(pdata + 1, info.device_ip);
    ResolveIpString(pdata + 5, info.subnet_mask);
    ResolveIpString(pdata + 9, info.gateway);
    ResolveMacAddress(pdata + 13, info.config_mac);
    ResolveIpString(pdata + 19, info.destination_ip);
    info.destination_port = *(pdata + 23) << 8 | *(pdata + 24);

    // check ret
    std::string str_chk =
        header_recv + data_recv.substr(0, data_recv.size() - 2);
    uint16_t val = get_check_sum(str_chk);
    uint16_t chk_sum = *(pdata + data_len - 2) << 8 | *(pdata + data_len - 1);
    if (val != chk_sum) {
      DisConnect();
      return DevAckError;
    }
  } else {
    DisConnect();
    return NotEnoughData;
  }

  // clear recv buffer
  int len = client_->GetAvailableBytesLen();
  if (len > 0) {
    std::string temp(len, 'x');
    client_->SyncRecv(temp, len);
  }

  return 0;
}

int LidarTools::GetMLXSDeviceCalibrationData(CalibrationData &cal) {
  // set send cmd
  const int send_len = 9;
  char mlxs_cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x00,
                             (char)0x00, (char)0x02, (char)0x0B,
                             (char)0x02, (char)0x00, (char)0x00};
  uint16_t chk_sum = get_check_sum(std::string(mlxs_cmd, 7));
  mlxs_cmd[7] = (chk_sum >> 8) & 0xFF;
  mlxs_cmd[8] = chk_sum & 0xFF;
  std::string cmd(mlxs_cmd, send_len);

  cal.device_type = zvision::DeviceType::LidarUnknown;
  cal.scan_mode = zvision::ScanMode::ScanUnknown;

  if (!CheckConnection())
    return TcpConnTimeout;

  if (client_->SyncSend(cmd, send_len)) {
    LOG_F(ERROR, "Send cali cmd error: %d", client_->GetSysErrorCode());
    DisConnect();
    return TcpSendTimeout;
  }
  const int recv_len = 7;
  std::string recv(recv_len, 'x');
  if (client_->SyncRecv(recv, recv_len)) {
    LOG_F(ERROR, "Receive cali ret error:  %d", client_->GetSysErrorCode());
    DisConnect();
    return TcpRecvTimeout;
  }

  // check ret
  bool check = true;
  {
    // check flg
    if (recv[4] != 0x00)
      check = false;
    // check sum
    if (check) {
      uint16_t val = get_check_sum(recv.substr(0, 5));
      uint16_t chk_sum = recv[5] << 8 | recv[6];
      check = (val == chk_sum);
    }
  }

  if (!check) {
    LOG_F(ERROR, "Check cali ret error: %d", client_->GetSysErrorCode());
    DisConnect();
    return DevAckError;
  }

  // get mlxs cali data
  uint32_t received_bytes_len = 0;
  uint32_t total_len = (36000 * 3 * 2);
  uint32_t total_bytes_len = total_len * 4;

  const int cali_header_len = 5;
  const int chk_sum_len = 2;
  std::vector<float> &data = cal.data;
  data.clear();
  // cali packet tail
  std::string cali_recv_left;
  while (received_bytes_len < total_bytes_len) {
    // receive calibration header
    std::string header_recv(cali_header_len, 'x');
    int ret = client_->SyncRecv(header_recv, cali_header_len);
    if (ret) {
      LOG_F(ERROR, "Receive mlxs calibration header error");
      DisConnect();
      return TcpRecvTimeout;
    }
    // chech cali header
    uint8_t *pheader_recv = (uint8_t *)header_recv.data();
    if ((*(pheader_recv + 0) != 0xBA) || (*(pheader_recv + 1) != 0xAC) ||
        (*(pheader_recv + 4) != 0x00)) {
      LOG_F(ERROR, "Check mlxs calibration header error.");
      DisConnect();
      return DevAckError;
    }

    // get cali data and checksum
    uint16_t data_len = 0;
    NetworkToHostShort((uint8_t *)header_recv.data() + 2, (char *)(&data_len));
    std::string data_recv(data_len, 'x');
    ret = client_->SyncRecv(data_recv, data_len);
    if (ret) {
      LOG_F(ERROR, "Receive mlxs calibration data error.");
      DisConnect();
      return TcpRecvTimeout;
    }

    // get packet check sum
    uint16_t pkt_chk = 0;
    NetworkToHostShort((uint8_t *)data_recv.data() + data_len - chk_sum_len,
                       (char *)(&pkt_chk));
    // get packet data
    std::string cali_recv = data_recv.substr(0, data_len - chk_sum_len);
    std::string cali_pkt = header_recv + cali_recv;
    // check
    uint16_t chk = get_check_sum(cali_pkt);
    if (chk != pkt_chk) {
      LOG_F(ERROR, "Check mlxs calibration data error.");
      DisConnect();
      return DevAckError;
    }

    // check cali data
    {
      unsigned char check_all_00 = 0x00;
      unsigned char check_all_ff = 0xFF;
      for (int i = 0; i < cali_recv.size(); i++) {
        check_all_00 |= cali_recv[i];
        check_all_ff &= cali_recv[i];
      }
      if (0x00 == check_all_00) {
        LOG_F(ERROR, "Check calibration data error, data is all 0x00.");
        DisConnect();
        return InvalidContent;
      }
      if (0xFF == check_all_ff) {
        LOG_F(ERROR, "Check calibration data error, data is all 0xFF.");
        DisConnect();
        return InvalidContent;
      }
    }
    // add data left over by last cali packet
    if (cali_recv_left.size()) {
      cali_recv = cali_recv_left + cali_recv;
      cali_recv_left = std::string();
    }
    // get cali packet tail
    int fcnt = cali_recv.size() / 4;
    int str_left = cali_recv.size() % 4;
    if (str_left > 0) {
      cali_recv_left = cali_recv.substr(fcnt * 4, str_left);
      cali_recv = cali_recv.substr(0, fcnt * 4);
    }

    // parsing cali data
    int network_data = 0;
    int host_data = 0;
    float *pfloat_data = reinterpret_cast<float *>(&host_data);
    unsigned char *pcali = (unsigned char *)cali_recv.c_str();
    for (int i = 0; i < fcnt; i++) {
      memcpy(&network_data, pcali + i * 4, 4);
      host_data = ntohl(network_data);
      data.push_back(*pfloat_data);
    }
    // check
    received_bytes_len += cali_recv.size();
    if (received_bytes_len >= total_bytes_len) {
      break;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(110));
  }

  // final check
  if (data.size() != total_len) {
    LOG_F(ERROR, "Get mlxs calibration data error.");
    return NotEnoughData;
  }

  cal.device_type = zvision::DeviceType::LidarMLX;
  cal.scan_mode = zvision::ScanMode::ScanMLXS_180;
  return 0;
}

int LidarTools::GetMLXSDeviceCalibrationDataToFile(std::string filename) {
  CalibrationData cal;
  int ret = GetMLXSDeviceCalibrationData(cal);

  if (ret)
    return ret;
  else
    return LidarTools::ExportCalibrationData(cal, filename);
}

int LidarTools::RebootMLXSDevice() {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 9;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x00,
                        (char)0x00, (char)0x02, (char)0x0C,
                        (char)0x01, (char)0x00, (char)0x00};
  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::ShutdownMLXSDevice() {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 9;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x00,
                        (char)0x00, (char)0x02, (char)0x0C,
                        (char)0x02, (char)0x00, (char)0x00};
  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

/* for ml30s plus a1 only */
int LidarTools::GetML30sPlusLogFile(std::string &log, bool isFull) {

  const int recv_len = 4;
  std::string recv(recv_len, 'x');
  // get log buffer length
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }
  uint32_t log_buffer_len = 0;
  NetworkToHost((const unsigned char *)recv.c_str(), (char *)&log_buffer_len);

  // get log buffer
  std::string log_buffer(log_buffer_len, 'x');
  if (client_->SyncRecv(log_buffer, log_buffer_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (isFull) {
    log = log_buffer;
    return 0;
  }

  // assemble the buffer to string
  unsigned char file_count = reinterpret_cast<unsigned char &>(log_buffer[0]);

  std::vector<std::string> file_contents;
  uint32_t position = 1;
  for (int i = 0; i < file_count; ++i) {
    uint32_t file_len = 0;
    NetworkToHost((const unsigned char *)(&log_buffer[position]),
                  (char *)&file_len);
    LOG_F(INFO, "FILE %d len %u.", i, file_len);
    // copy to file list
    std::string content(log_buffer, position + 4, file_len);
    file_contents.push_back(content);
    position += (file_len + 4);
  }

  // reorder the log file
  log = "";
  for (int i = file_count - 1; i >= 0; i--)
    log += file_contents[i];

  recv.resize(4, 'x');
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }
  // check ret
  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }
  return 0;
}

int LidarTools::GenerateML30SPlusDeviceCmdString(
    EML30SPlusCmd type, std::string &buf, zvision::JsonConfigFileParam *param) {

  if (!param)
    return InvalidParameter;
  param->temp_recv_data_len = 0;

  std::string str_cmd;
  std::string str_value;
  bool valid = true;
  switch (type) {
  // set lidar config
  case zvision::set_dhcp_switch: {
    char cmd[3] = {(char)0xBA, (char)0x02, (char)0x07};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->dhcp_switch, str_value);
  } break;
  case zvision::set_ip: {
    char cmd[3] = {(char)0xBA, (char)0x02, (char)0x01};
    str_cmd.append(cmd, 3);
    char ip[4] = {0};
    if (!AssembleIpString(param->ip, ip))
      return InvalidParameter;
    str_value = std::string(ip, 6);
  } break;
  case zvision::set_gateway: {
    char cmd[3] = {(char)0xBA, (char)0x02, (char)0x02};
    str_cmd.append(cmd, 3);
    char ip[4] = {0};
    if (!AssembleIpString(param->gateway, ip))
      return InvalidParameter;
    str_value = std::string(ip, 6);
  } break;
  case zvision::set_netmask: {
    char cmd[3] = {(char)0xBA, (char)0x02, (char)0x03};
    str_cmd.append(cmd, 3);
    char ip[4] = {0};
    if (!AssembleIpString(param->netmask, ip))
      return InvalidParameter;
    str_value = std::string(ip, 6);
  } break;
  case zvision::set_mac: {
    char cmd[3] = {(char)0xBA, (char)0x02, (char)0x04};
    str_cmd.append(cmd, 3);
    char mac[6] = {0};
    if (!AssembleMacAddress(param->mac, mac))
      return InvalidParameter;
    str_value = std::string(mac, 6);
  } break;
  case zvision::set_udp_dest_ip: {
    char cmd[3] = {(char)0xBA, (char)0x02, (char)0x05};
    str_cmd.append(cmd, 3);
    char ip[4] = {0};
    if (!AssembleIpString(param->udp_dest_ip, ip))
      return InvalidParameter;
    str_value = std::string(ip, 6);
  } break;
  case zvision::set_udp_dest_port: {
    char cmd[3] = {(char)0xBA, (char)0x02, (char)0x06};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->udp_dest_port, str_value);
  } break;
  case zvision::set_near_point_delete_switch: {
    char cmd[3] = {(char)0xBA, (char)0x03, (char)0x01};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->switch_near_point_delete, str_value);
  } break;
  case zvision::set_retro_switch: {
    char cmd[3] = {(char)0xBA, (char)0x04, (char)0x01};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->switch_retro, str_value);
  } break;
  case zvision::set_retro_target_gray_thre: {
    char cmd[3] = {(char)0xBA, (char)0x04, (char)0x03};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->target_gray_thre_retro, str_value);
  } break;
  case zvision::set_retro_target_point_num_thre: {
    char cmd[3] = {(char)0xBA, (char)0x04, (char)0x02};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->target_point_num_thre_retro, str_value);
  } break;
  case zvision::set_retro_critical_point_dis_thre: {
    char cmd[3] = {(char)0xBA, (char)0x04, (char)0x04};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->critical_point_dis_thre_retro, str_value);
  } break;
  case zvision::set_retro_del_point_dis_low_thre: {
    char cmd[3] = {(char)0xBA, (char)0x04, (char)0x05};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->del_point_dis_low_thre_retro, str_value);
  } break;
  case zvision::set_retro_del_point_dis_high_thre: {
    char cmd[3] = {(char)0xBA, (char)0x04, (char)0x06};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->del_point_dis_high_thre_retro, str_value);
  } break;
  case zvision::set_retro_del_point_gray_thre: {
    char cmd[3] = {(char)0xBA, (char)0x04, (char)0x07};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->del_point_gray_thre_retro, str_value);
  } break;
  case zvision::set_adhesion_switch: {
    char cmd[3] = {(char)0xBA, (char)0x05, (char)0x01};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->switch_adhesion, str_value);
  } break;
  case zvision::set_adhesion_angle_hor_min: {
    char cmd[3] = {(char)0xBA, (char)0x05, (char)0x02};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->angle_hor_min_adhesion, str_value);
  } break;
  case zvision::set_adhesion_angle_hor_max: {
    char cmd[3] = {(char)0xBA, (char)0x05, (char)0x03};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->angle_hor_max_adhesion, str_value);
  } break;
  case zvision::set_adhesion_angle_ver_min: {
    char cmd[3] = {(char)0xBA, (char)0x05, (char)0x04};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->angle_ver_min_adhesion, str_value);
  } break;
  case zvision::set_adhesion_angle_ver_max: {
    char cmd[3] = {(char)0xBA, (char)0x05, (char)0x05};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->angle_ver_max_adhesion, str_value);
  } break;
  case zvision::set_adhesion_angle_hor_res: {
    char cmd[3] = {(char)0xBA, (char)0x05, (char)0x06};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->angle_hor_res_adhesion, str_value);
  } break;
  case zvision::set_adhesion_angle_ver_res: {
    char cmd[3] = {(char)0xBA, (char)0x05, (char)0x07};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->angle_ver_res_adhesion, str_value);
  } break;
  case zvision::set_adhesion_diff_thre: {
    char cmd[3] = {(char)0xBA, (char)0x05, (char)0x08};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->diff_thre_adhesion, str_value);
  } break;
  case zvision::set_adhesion_dist_limit: {
    char cmd[3] = {(char)0xBA, (char)0x05, (char)0x09};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->dist_limit_adhesion, str_value);
  } break;
  case zvision::set_adhesion_min_diff: {
    char cmd[3] = {(char)0xBA, (char)0x05, (char)0x0A};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->min_diff_adhesion, str_value);
  } break;
  case zvision::set_down_sample_mode: {
    char cmd[3] = {(char)0xBA, (char)0x06, (char)0x01};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->down_sample_mode, str_value);
  } break;
  case zvision::set_dirty_detect_switch: {
    char cmd[3] = {(char)0xBA, (char)0x07, (char)0x01};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->switch_dirty_detect, str_value);
  } break;
  case zvision::set_dirty_detect_refresh: {

  } break;
  case zvision::set_dirty_detect_cycle: {

  } break;
  case zvision::set_dirty_detect_set_thre: {

  } break;
  case zvision::set_dirty_detect_reset_thre: {

  } break;
  case zvision::set_dirty_detect_inner_thre: {

  } break;
  case zvision::set_dirty_detect_outer_thre: {

  } break;
  case zvision::set_echo_mode: {
    char cmd[3] = {(char)0xBA, (char)0x08, (char)0x01};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->echo_mode, str_value);
  } break;
  case zvision::set_ptp_sync: {
    char cmd[3] = {(char)0xBA, (char)0x09, (char)0x01};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->ptp_sync, str_value);
  } break;
  case zvision::set_ptp_file: {
    char cmd[3] = {(char)0xBA, (char)0x09, (char)0x02};
    str_cmd.append(cmd, 3);
    // read file data
    std::ifstream infile(param->temp_filepath);
    if (!infile.is_open())
      return OpenFileError;
    std::stringstream ss;
    ss << infile.rdbuf();
    param->temp_send_data = ss.str();
    param->temp_send_data_len = param->temp_send_data.size();
    GenerateStringFromValue(param->temp_send_data_len, str_value);
  } break;
  case zvision::set_frame_sync: {
    char cmd[3] = {(char)0xBA, (char)0x0A, (char)0x01};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->frame_sync, str_value);
  } break;
  case zvision::set_frame_offset: {
    char cmd[3] = {(char)0xBA, (char)0x0A, (char)0x02};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->frame_offset, str_value);
  } break;
  case zvision::set_angle_send: {
    char cmd[3] = {(char)0xBA, (char)0x0B, (char)0x01};
    str_cmd.append(cmd, 3);
    GenerateStringFromValue(param->angle_send, str_value);
  } break;
  case zvision::set_json_config_file: {
    char cmd[3] = {(char)0xBA, (char)0x0E, (char)0x01};
    str_cmd.append(cmd, 3);
    // read file data
    std::ifstream infile(param->temp_filepath);
    if (!infile.is_open())
      return OpenFileError;
    std::stringstream ss;
    ss << infile.rdbuf();
    std::string str = ss.str();
    // check json format
    rapidjson::Document doc;
    if (doc.Parse(str.c_str()).HasParseError())
      return InvalidContent;

    param->temp_send_data = str;
    param->temp_send_data_len = str.size();
    GenerateStringFromValue(param->temp_send_data_len, str_value);
  } break;
  // lidar control
  case zvision::reboot: {
    char cmd[4] = {(char)0xBA, (char)0x0c, (char)0x01, (char)0x01};
    str_cmd.append(cmd, 4);
  } break;

  // get lidar config
  case zvision::read_serial_number: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x01, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = 0;
  } break;
  case zvision::read_factory_mac: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x02, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = 8;
  } break;
  case zvision::read_network_param: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x03, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = 27;
  } break;
  case zvision::read_embeded_fpga_version: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x04, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = 16;
  } break;
  case zvision::read_config_param_in_use: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x05, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = -2; // 50;
  } break;
  case zvision::read_config_param_saved: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x06, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = -2; // 50;
  } break;
  case zvision::read_algo_param_in_use: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x07, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = -2; // 84;
  } break;
  case zvision::read_algo_param_saved: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x08, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = -2; // 84;
  } break;
  case zvision::read_json_config_file: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x09, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = -4;
  } break;
  case zvision::read_cali_packets: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x0A, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = 0;
  } break;
  case zvision::read_temp_log: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x0B, (char)0x00};
    str_cmd.append(cmd, 4);
    // special deal
    param->temp_recv_data_len = 0;
  } break;
  case zvision::read_full_log: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x0C, (char)0x00};
    str_cmd.append(cmd, 4);
    // special deal
    param->temp_recv_data_len = 0;
  } break;
  case read_ptp_file: {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x0D, (char)0x00};
    str_cmd.append(cmd, 4);
    param->temp_recv_data_len = -4;
  } break;

  default:
    return InvalidParameter;
  }

  std::string str = str_cmd + str_value;
  if (!valid || str.empty())
    return InvalidContent;

  buf = str;
  return 0;
}

int LidarTools::RunPost(EML30SPlusCmd type,
                        zvision::JsonConfigFileParam *param) {

  if (!param)
    return InvalidParameter;

  const unsigned char *pdata = (unsigned char *)param->temp_recv_data.c_str();
  const int data_len = param->temp_recv_data.size();
  switch (type) {
  case zvision::read_serial_number:
    param->serial_number = param->temp_recv_data;
    break;
  case zvision::read_factory_mac: {
    char fmac[128] = "";
    sprintf_s(fmac, "%02X%02X%02X%02X%02X%02X", pdata[2], pdata[3], pdata[4],
              pdata[5], pdata[6], pdata[7]);
    param->factory_mac = std::string(fmac);
  } break;
  case zvision::read_network_param: {
    param->dhcp_switch = *(pdata + 0);
    ResolveIpString(pdata + 1, param->ip);
    ResolveIpString(pdata + 5, param->netmask);
    ResolveIpString(pdata + 9, param->gateway);
    ResolveIpString(pdata + 13, param->mac);
    ResolveIpString(pdata + 19, param->udp_dest_ip);
    int port = 0;
    ResolvePort(pdata + 23, port);
    param->udp_dest_port = port;
  } break;
  case zvision::read_embeded_fpga_version: {
    memcpy(param->embedded_ver, pdata, 4);
    memcpy(param->fpga_ver, pdata + 4, 4);
    ResolveIpString(pdata, param->embedded_version);
    ResolveIpString(pdata + 4, param->fpga_version);

    memcpy(param->embedded_ver_bak, pdata + 8, 4);
    memcpy(param->fpga_ver_bak, pdata + 12, 4);
    ResolveIpString(pdata + 8, param->embedded_version_bak);
    ResolveIpString(pdata + 12, param->fpga_version_bak);
  } break;
  case zvision::read_config_param_in_use:
  case zvision::read_config_param_saved: {
    // NetworkToHostShort(pdata,(char*)&param->algo_param_len);
    if (data_len < 48)
      return InvalidContent;
    memcpy(param->embedded_ver, pdata, 4);
    ResolveIpString(pdata, param->embedded_version);
    memcpy(param->fpga_ver, pdata + 4, 4);
    ResolveIpString(pdata + 4, param->fpga_version);
    ResolveIpString(pdata + 8, param->ip);
    ResolveIpString(pdata + 12, param->gateway);
    ResolveIpString(pdata + 16, param->netmask);
    ResolveMacAddress(pdata + 20, param->mac);
    ResolveIpString(pdata + 26, param->udp_dest_ip);
    NetworkToHost(pdata + 30, (char *)&param->udp_dest_port);
    param->dhcp_switch = *(pdata + 34);
    param->switch_retro = *(pdata + 35);
    param->switch_near_point_delete = *(pdata + 36);
    param->switch_adhesion = *(pdata + 37);
    param->down_sample_mode = *(pdata + 38);
    param->echo_mode = *(pdata + 39);
    param->ptp_sync = *(pdata + 40);
    param->switch_dirty_detect = *(pdata + 41);
    param->angle_send = *(pdata + 42);
    param->frame_sync = *(pdata + 43);
    NetworkToHost(pdata + 44, (char *)&param->frame_offset);
  } break;
  case zvision::read_algo_param_in_use:
  case zvision::read_algo_param_saved: {

    // parameter data
    const unsigned char *param_data = pdata + 20;
    uint16_t param_len = data_len - 20;
    // before 20 bytes is header data
    // near point delete
    uint16_t offset = 0;
    uint16_t len = 0;
    {
      NetworkToHostShort(pdata, (char *)&offset);
      NetworkToHostShort(pdata + 2, (char *)&len);
      if (len >= 1 && ((offset + len) <= param_len)) {
        const unsigned char *param_del = param_data + offset;
        param->switch_near_point_delete = *(param_del + 0);
      }
    }
    // retro
    {
      NetworkToHostShort(pdata + 4, (char *)&offset);
      NetworkToHostShort(pdata + 6, (char *)&len);
      if (len >= 12 && ((offset + len) <= param_len)) {
        const unsigned char *param_retro = param_data + offset;
        param->switch_retro = *(param_retro + 0);
        param->target_gray_thre_retro = *(param_retro + 1);
        param->target_point_num_thre_retro = *(param_retro + 2);
        NetworkToHost(param_retro + 3,
                      (char *)&param->critical_point_dis_thre_retro);
        NetworkToHostShort(param_retro + 7,
                           (char *)&param->del_point_dis_low_thre_retro);
        NetworkToHostShort(param_retro + 9,
                           (char *)&param->del_point_dis_high_thre_retro);
        param->del_point_gray_thre_retro = *(param_retro + 11);
      }
    }

    // adhesion
    {
      NetworkToHostShort(pdata + 8, (char *)&offset);
      NetworkToHostShort(pdata + 10, (char *)&len);
      if (len >= 37 && ((offset + len) <= param_len)) {
        const unsigned char *param_adhesion = param_data + offset;
        param->switch_adhesion = *(param_adhesion + 0);
        NetworkToHost(param_adhesion + 1,
                      (char *)&param->angle_hor_min_adhesion);
        NetworkToHost(param_adhesion + 5,
                      (char *)&param->angle_hor_max_adhesion);
        NetworkToHost(param_adhesion + 9,
                      (char *)&param->angle_ver_min_adhesion);
        NetworkToHost(param_adhesion + 13,
                      (char *)&param->angle_ver_max_adhesion);
        NetworkToHost(param_adhesion + 17,
                      (char *)&param->angle_hor_res_adhesion);
        NetworkToHost(param_adhesion + 21,
                      (char *)&param->angle_ver_res_adhesion);
        NetworkToHost(param_adhesion + 25, (char *)&param->diff_thre_adhesion);
        NetworkToHost(param_adhesion + 29, (char *)&param->dist_limit_adhesion);
        NetworkToHost(param_adhesion + 33, (char *)&param->min_diff_adhesion);
      }
    }

    // downsample
    {
      NetworkToHostShort(pdata + 12, (char *)&offset);
      NetworkToHostShort(pdata + 14, (char *)&len);
      if (len >= 1 && ((offset + len) <= param_len)) {
        const unsigned char *param_downsample = param_data + offset;
        param->down_sample_mode = *(param_downsample + 0);
      }
    }

    // dirty detect
    {
      NetworkToHostShort(pdata + 16, (char *)&offset);
      NetworkToHostShort(pdata + 18, (char *)&len);
      if (len >= 12 && ((offset + len) <= param_len)) {
        const unsigned char *param_dirty = param_data + offset;
        param->switch_dirty_detect = *(param_dirty + 0);
        param->switch_dirty_refresh = *(param_dirty + 1);
        NetworkToHostShort(param_dirty + 2,
                           (char *)&param->dirty_refresh_cycle);
        NetworkToHostShort(param_dirty + 4,
                           (char *)&param->dirty_detect_set_thre);
        NetworkToHostShort(param_dirty + 6,
                           (char *)&param->dirty_detect_reset_thre);
        NetworkToHostShort(param_dirty + 8,
                           (char *)&param->dirty_detect_inner_thre);
        NetworkToHostShort(param_dirty + 10,
                           (char *)&param->dirty_detect_outer_thre);
      }
    }
  } break;
  case zvision::read_json_config_file: {

  } break;
  case zvision::read_temp_log: {

  } break;
  case zvision::read_full_log: {

  } break;
  case zvision::read_ptp_file: {
    std::fstream outfile;
    outfile.open(param->temp_filepath, std::ios::out);
    if (outfile.is_open() && param->temp_recv_data.size()) {
      outfile << param->temp_recv_data;
      outfile.close();
    } else {
      return OpenFileError;
    }
  } break;
  default:
    break;
  }

  return 0;
}

int LidarTools::RunML30sPlusDeviceManager(EML30SPlusCmd type,
                                          zvision::JsonConfigFileParam *param) {

  // pre check
  if (!param)
    return InvalidParameter;

  /* for special command */
  // read cali packets
  if (type == read_cali_packets) {
    char cmd[4] = {(char)0xBA, (char)0x0D, (char)0x0A, (char)0x00};
    return GetDeviceCalibrationPackets(param->temp_recv_packets,
                                       std::string(cmd, 4));
  }

  /*  1. We generate tcp command , send data if in need. */
  std::string cmd_str;
  int ret = GenerateML30SPlusDeviceCmdString(type, cmd_str, param);
  if (ret != 0)
    return InvalidParameter;

  /* 2. Send cmd to device. */
  // Check tcp connection.
  if (!CheckConnection())
    return TcpConnTimeout;
  // send cmd
  if (client_->SyncSend(cmd_str, cmd_str.size())) {
    DisConnect();
    return TcpSendTimeout;
  }

  // need ack
  if (type != set_json_config_file) {
    const int recv_len = 4;
    std::string recv(4, 'x');
    if (client_->SyncRecv(recv, recv_len)) {
      DisConnect();
      return TcpRecvTimeout;
    }

    if (!CheckDeviceRet(recv)) {
      DisConnect();
      return DevAckError;
    }
  }

  // If don`t send or receive data, return.
  if (type < cmd_section_control)
    return 0;

  /* 3. for special command */
  // read log ...
  if (type == read_temp_log || type == read_full_log) {
    return GetML30sPlusLogFile(param->temp_recv_data, type == read_full_log);
  }

  // We send/receive 1024 bytes once.
  int pkt_len = 1024;
  /* 4.  Send data to device. */
  if (type > cmd_section_control && type < cmd_section_send_data) {
    int send_len = param->temp_send_data.size();
    if (!send_len)
      return InvalidContent;

    std::string send_data = param->temp_send_data;
    int pkt_cnt = send_len / pkt_len;
    if (send_len % pkt_len)
      pkt_cnt++;
    for (int i = 0; i < pkt_cnt; i++) {

      int len = pkt_len;
      if (i == send_len / pkt_len)
        len = send_len % pkt_len;

      // send
      std::string str_send(send_data.c_str() + i * pkt_len, len);
      if (client_->SyncSend(str_send, len)) {
        DisConnect();
        return TcpSendTimeout;
      }

      // update
      if (i > 5)
        std::this_thread::sleep_for(std::chrono::microseconds(110));
    }

    // 4.1 check ret.
    {
      const int recv_len = 4;
      std::string recv(4, 'x');
      if (client_->SyncRecv(recv, recv_len)) {
        DisConnect();
        return TcpRecvTimeout;
      }
      if (!CheckDeviceRet(recv)) {
        DisConnect();
        return DevAckError;
      }
    }
  }
  /* 5.  Receive data from device. */
  else if (type > cmd_section_send_data && type < cmd_section_receive_data) {

    std::string recv_data;
    uint32_t recv_len = 0;
    // Get recv data length.
    if (param->temp_recv_data_len < 0) {
      int bytes = std::abs(param->temp_recv_data_len);
      std::string str(bytes, 'x');
      if (client_->SyncRecv(str, bytes)) {
        DisConnect();
        return TcpRecvTimeout;
      }

      // Get length.
      if (bytes == 1)
        recv_len = (uint8_t)str[0];
      else if (bytes == 2) {
        uint16_t val = 0;
        NetworkToHostShort((uint8_t *)str.c_str(), (char *)&val);
        recv_len = val;
      } else if (bytes == 4)
        NetworkToHost((uint8_t *)str.c_str(), (char *)&recv_len);
      else
        return NotMatched;
    } else if (param->temp_recv_data_len == 0)
      recv_len = client_->GetAvailableBytesLen();
    else
      recv_len = param->temp_recv_data_len;

    if (recv_len == 0)
      return NotEnoughData;

    // for special cmd
    if (type == read_algo_param_in_use || type == read_algo_param_saved) {
      // 20 bytes header
      recv_len += 20;
    }

    // Read data.
    int pkt_cnt = recv_len / pkt_len;
    if (recv_len % pkt_len)
      pkt_cnt++;
    for (int i = 0; i < pkt_cnt; i++) {

      int len = pkt_len;
      if (i == recv_len / pkt_len)
        len = recv_len % pkt_len;

      // recv ret
      std::string pkt(len, 'x');
      if (client_->SyncRecv(pkt, len)) {
        DisConnect();
        return TcpRecvTimeout;
      }
      // update
      const char *pdata = pkt.data();
      recv_data += std::string(pdata, len);
      if (i > 10)
        std::this_thread::sleep_for(std::chrono::microseconds(110));
    }

    param->temp_recv_data = recv_data;
  }

  // 6. post deal
  if (RunPost(type, param) != 0)
    return Failure;

  // read all recv buf if necessar
  {
    int len = client_->GetAvailableBytesLen();
    if (len > 0) {
      std::string tmp(len, 'x');
      client_->SyncRecv(tmp, len);
    }
  }

  return 0;
}

int LidarTools::QueryML30sPlusDeviceConfigurationInfo(
    DeviceConfigurationInfo &info) {

  int ret = 0;
  JsonConfigFileParam param;
  ret = RunML30sPlusDeviceManager(EML30SPlusCmd::read_config_param_in_use,
                                  &param);
  if (ret != 0)
    return ret;

  ret = RunML30sPlusDeviceManager(EML30SPlusCmd::read_embeded_fpga_version,
                                  &param);
  if (ret != 0)
    return ret;

  info.device = DeviceType::LidarMl30SA1Plus;
  info.serial_number = param.serial_number;
  memcpy(info.version.boot_version, param.fpga_ver, 4);
  memcpy(info.version.kernel_version, param.embedded_ver, 4);
  memcpy(info.backup_version.boot_version, param.fpga_ver_bak, 4);
  memcpy(info.backup_version.kernel_version, param.embedded_ver_bak, 4);
  info.config_mac = param.mac;
  info.device_mac = param.mac;
  info.factory_mac = param.factory_mac;
  info.device_ip = param.ip;
  info.subnet_mask = param.netmask;
  info.destination_ip = param.udp_dest_ip;
  info.destination_port = param.udp_dest_port;
  info.time_sync = TimestampType::TimestampPtp;
  info.phase_offset = param.frame_offset;
  info.phase_offset_mode = PhaseOffsetMode::PhaseOffsetUnknown;
  if (param.frame_sync == 0)
    info.phase_offset_mode = PhaseOffsetMode::PhaseOffsetDisable;
  else if (param.frame_sync == 1)
    info.phase_offset_mode = PhaseOffsetMode::PhaseOffsetEnable;

  info.echo_mode = EchoMode::EchoUnknown;
  if (param.echo_mode == 1)
    info.echo_mode = EchoMode::EchoSingleFirst;
  else if (param.echo_mode == 2)
    info.echo_mode = EchoMode::EchoSingleStrongest;
  else if (param.echo_mode == 3)
    info.echo_mode = EchoMode::EchoSingleLast;
  else if (param.echo_mode == 4)
    info.echo_mode = EchoMode::EchoDoubleFirstStrongest;
  else if (param.echo_mode == 5)
    info.echo_mode = EchoMode::EchoDoubleFirstLast;
  else if (param.echo_mode == 6)
    info.echo_mode = EchoMode::EchoDoubleStrongestLast;

  info.cal_send_mode = CalSendMode::CalSendUnknown;
  if (param.angle_send == 0)
    info.cal_send_mode = CalSendMode::CalSendDisable;
  else if (param.angle_send == 1)
    info.cal_send_mode = CalSendMode::CalSendEnable;

  info.downsample_mode = DownsampleMode::DownsampleUnknown;
  if (param.down_sample_mode == 0)
    info.downsample_mode = DownsampleMode::DownsampleNone;
  else if (param.switch_near_point_delete == 1)
    info.downsample_mode = DownsampleMode::Downsample_1_2;
  else if (param.switch_near_point_delete == 2)
    info.downsample_mode = DownsampleMode::Downsample_1_4;

  info.dhcp_enable = StateMode::StateUnknown;
  if (param.dhcp_switch == 0)
    info.dhcp_enable = StateMode::StateDisable;
  else if (param.dhcp_switch == 1)
    info.dhcp_enable = StateMode::StateEnable;

  info.gateway_addr = param.gateway;

  // return 0;

  // serial number
  ret = RunML30sPlusDeviceManager(EML30SPlusCmd::read_serial_number, &param);
  if (ret != 0)
    return ret;

  info.serial_number = param.serial_number;

  // algo param section
  info.algo_param.isValid = false;
  info.adhesion_enable = StateMode::StateUnknown;
  info.retro_enable = RetroMode::RetroUnknown;
  info.delete_point_enable = StateMode::StateUnknown;
  ret =
      RunML30sPlusDeviceManager(EML30SPlusCmd::read_algo_param_in_use, &param);
  if (ret != 0)
    return ret;

  info.algo_param.isValid = true;
  if (param.switch_adhesion == 0)
    info.adhesion_enable = StateMode::StateDisable;
  else if (param.switch_adhesion == 1)
    info.adhesion_enable = StateMode::StateEnable;

  if (param.switch_retro == 0)
    info.retro_enable = RetroMode::RetroDisable;
  else if (param.switch_retro == 1)
    info.retro_enable = RetroMode::RetroEnable;

  if (param.switch_near_point_delete == 0)
    info.delete_point_enable = StateMode::StateDisable;
  else if (param.switch_near_point_delete == 1)
    info.delete_point_enable = StateMode::StateEnable;

  info.algo_param.isValid = true;
  info.algo_param.adhesion_angle_hor_max = param.angle_hor_max_adhesion;
  info.algo_param.adhesion_angle_hor_min = param.angle_hor_min_adhesion;
  info.algo_param.adhesion_angle_hor_res = param.angle_hor_res_adhesion;
  info.algo_param.adhesion_angle_ver_max = param.angle_ver_max_adhesion;
  info.algo_param.adhesion_angle_ver_min = param.angle_ver_min_adhesion;
  info.algo_param.adhesion_angle_ver_res = param.angle_ver_res_adhesion;
  info.algo_param.adhesion_diff_thres = param.diff_thre_adhesion;
  info.algo_param.adhesion_dis_limit = param.dist_limit_adhesion;
  info.algo_param.adhesion_min_diff = param.min_diff_adhesion;

  info.algo_param.retro_del_gray_thres = param.del_point_gray_thre_retro;
  info.algo_param.retro_del_ratio_gray_high_thres = 0;
  info.algo_param.retro_del_ratio_gray_low_thres = 0;
  info.algo_param.retro_dis_thres = param.critical_point_dis_thre_retro;
  info.algo_param.retro_high_range_thres = param.del_point_dis_high_thre_retro;
  info.algo_param.retro_low_range_thres = param.del_point_dis_low_thre_retro;
  info.algo_param.retro_min_gray = param.target_gray_thre_retro;
  info.algo_param.retro_min_gray_num = param.target_point_num_thre_retro;

  // read factory mac
  ret = RunML30sPlusDeviceManager(EML30SPlusCmd::read_factory_mac, &param);
  if (ret != 0)
    return ret;
  info.factory_mac = param.factory_mac;
  return 0;
}

int LidarTools::ML30sPlusFirmwareUpdate(std::string &filename,
                                        ProgressCallback cb, bool isBak) {
  if (!CheckConnection())
    return TcpConnTimeout;

  std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
  if (!in.is_open()) {
    LOG_F(ERROR, "Open file error.");
    return OpenFileError;
  }

  std::streampos end = in.tellg();
  int size = static_cast<int>(end);
  in.close();

  const int send_len = 7;
  char upgrade_cmd[send_len] = {(char)0xBA, (char)0x01, (char)0x01, (char)0x00,
                                (char)0x00, (char)0x00, (char)0x00};
  if (isBak)
    upgrade_cmd[2] = 0x02;

  HostToNetwork((const unsigned char *)&size, upgrade_cmd + 3);
  std::string cmd(upgrade_cmd, send_len);

  const int recv_len = 4;
  std::string recv(recv_len, 'x');

  if (client_->SyncSend(cmd, send_len)) {
    DisConnect();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  // transfer, erase flash, write
  const int pkt_len = 256;
  int start_percent = 10;
  int block_total = size / pkt_len;
  if (0 != (size % pkt_len))
    block_total += 1;
  int step_per_percent = block_total / 30;

  std::ifstream idata(filename, std::ios::in | std::ios::binary);
  char fw_data[pkt_len] = {0x00};
  int readed = 0;

  // data transfer
  if (idata.is_open()) {
    for (readed = 0; readed < block_total; readed++) {
      int read_len = pkt_len;
      if ((readed == (block_total - 1)) && (0 != (size % pkt_len)))
        read_len = size % pkt_len;
      idata.read(fw_data, read_len);
      std::string fw(fw_data, read_len);
      if (client_->SyncSend(fw, read_len)) {
        break;
      }
      if (0 == (readed % step_per_percent)) {
        cb(start_percent++, this->device_ip_.c_str());
      }
    }
    idata.close();
  }
  if (readed != block_total) {
    client_->Close();
    return TcpSendTimeout;
  }

  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }
  LOG_F(2, "Data transfer ok...");

  // waitting for step 2
  const int recv_step_len = 5;
  std::string recv_step(recv_step_len, 'x');
  bool ok = false;
  while (1) {
    if (client_->SyncRecv(recv_step, recv_step_len)) {
      ok = false;
      break;
    }
    unsigned char step = (unsigned char)recv_step[4];
    cb(40 + int((double)step / 3.3), this->device_ip_.c_str());
    if (100 == step) {
      ok = true;
      break;
    }
  }

  if (!ok) {
    LOG_F(ERROR, "Waiting for erase flash failed...");
    DisConnect();
    return TcpRecvTimeout;
  }
  LOG_F(2, "Waiting for erase flash ok...");

  // waitting for step 3
  while (1) {
    if (client_->SyncRecv(recv_step, recv_step_len)) {
      ok = false;
      break;
    }
    unsigned char step = (unsigned char)recv_step[4];
    cb(70 + int((double)step / 3.3), this->device_ip_.c_str());
    if (100 == step) {
      ok = true;
      break;
    }
  }

  if (!ok) {
    LOG_F(ERROR, "Waiting for write flash failed...");
    DisConnect();
    return TcpRecvTimeout;
  }
  LOG_F(2, "Waiting for write flash ok...");

  // device check data
  if (client_->SyncRecv(recv, recv_len)) {
    DisConnect();
    return TcpRecvTimeout;
  }

  if (!CheckDeviceRet(recv)) {
    DisConnect();
    return DevAckError;
  }

  return 0;
}

/* for ml30s plus b1 only */
int LidarTools::GetML30sPlusB1DeviceRet(std::string &out) {
  const int chk_sum_len = 2;
  const int recv_header_len = 5;
  // get ret header
  std::string recv_header(recv_header_len, 'x');
  if (client_->SyncRecv(recv_header, recv_header_len)) {
    LOG_F(ERROR, "Receive recv header error:  %d.", client_->GetSysErrorCode());
    DisConnect();
    return TcpRecvTimeout;
  }
  uint16_t recv_data_len = 0;
  NetworkToHostShort((uint8_t *)recv_header.data() + 2,
                     (char *)(&recv_data_len));

  // recv data error
  if (recv_data_len < chk_sum_len) {
    int len = client_->GetAvailableBytesLen();
    if (len > 0) {
      std::string temp(len, 'x');
      client_->SyncRecv(temp, len);
    }
    LOG_F(ERROR, "Check recv header error.");
    DisConnect();
    return DevAckError;
  }

  // get ret data
  std::string recv_data(recv_data_len, 'x');
  if (client_->SyncRecv(recv_data, recv_data_len)) {
    LOG_F(ERROR, "Receive recv data error:  %d.", client_->GetSysErrorCode());
    DisConnect();
    return TcpRecvTimeout;
  }

  out = recv_header + recv_data;
  return 0;
}

int LidarTools::CheckML30sPlusB1DeviceRet(std::string ret) {
  const int min_ret_len = 7;
  if (ret.size() < min_ret_len)
    return DevAckError;

  const int chk_sum_len = 2;
  // check sum
  uint8_t *pdata = (uint8_t *)ret.data();
  uint16_t chk_sum = get_check_sum(ret.substr(0, ret.size() - chk_sum_len));
  uint16_t chk_val = *(pdata + ret.size() - chk_sum_len) << 8 |
                     *(pdata + ret.size() - chk_sum_len + 1);
  if (chk_sum != chk_val)
    return DevAckError;

  // check flag
  return *(pdata + 4);
}

int LidarTools::GetML30sPlusB1DeviceRetData(std::string &out) {
  std::string str_ret;
  int ret = GetML30sPlusB1DeviceRet(str_ret);
  if (ret)
    return ret;

  ret = CheckML30sPlusB1DeviceRet(str_ret);
  if (ret)
    return ret;

  const int min_ret_len = 7;
  const int ret_header_len = 5;
  const int chk_sum_len = 2;
  if (str_ret.size() < min_ret_len)
    return NotEnoughData;

  out = str_ret.substr(ret_header_len,
                       str_ret.size() - ret_header_len - chk_sum_len);
  return 0;
}

int LidarTools::GetML30sPlusB1DeviceQueryString(const std::string &cmd,
                                                std::string &str) {
  const int chk_sum_len = 2;
  if (cmd.size() < chk_sum_len) {
    DisConnect();
    return InvalidParameter;
  }

  std::string cmd_str = cmd;
  std::string chk_str = cmd.substr(0, cmd.size() - 2);
  uint16_t chk_sum = get_check_sum(chk_str);
  cmd_str.at(cmd_str.size() - chk_sum_len) = char((chk_sum >> 8) & 0xFF);
  cmd_str.at(cmd_str.size() - chk_sum_len + 1) = char(chk_sum & 0xFF);

  if (client_->SyncSend(cmd_str, cmd_str.size())) {
    DisConnect();
    return TcpSendTimeout;
  }

  std::string data_str;
  int ret = GetML30sPlusB1DeviceRetData(data_str);
  if (ret)
    return ret;

  str = data_str;
  return 0;
}

int LidarTools::GetML30sPlusB1DeviceCalibrationPackets(
    CalibrationPackets &pkts) {
  if (!CheckConnection())
    return TcpConnTimeout;

  // set send cmd
  const int send_len = 9;
  // char mlxs_cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01,
  //                            (char)0x00, (char)0x02, (char)0x0B,
  //                            (char)0x02, (char)0x00, (char)0x00};
  char mlxs_cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x0,
                             (char)0x00, (char)0x02, (char)0x0B,
                             (char)0x02, (char)0x00, (char)0x00};
  uint16_t chk_sum = get_check_sum(std::string(mlxs_cmd, 7));
  mlxs_cmd[7] = (chk_sum >> 8) & 0xFF;
  mlxs_cmd[8] = chk_sum & 0xFF;
  std::string cmd(mlxs_cmd, send_len);
  // send cmd
  if (client_->SyncSend(cmd, send_len)) {
    LOG_F(ERROR, "Send cali cmd error: %d", client_->GetSysErrorCode());
    DisConnect();
    return TcpSendTimeout;
  }
  // get ret part 1
  const int ret_len1 = 5;
  std::string recv_ret1(ret_len1, 'x');
  if (client_->SyncRecv(recv_ret1, ret_len1)) {
    LOG_F(ERROR, "Receive cali ret1 error:  %d", client_->GetSysErrorCode());
    DisConnect();
    return TcpRecvTimeout;
  }
  // get ret part 2 len
  uint16_t ret_len2 = 0;
// zvision:
  NetworkToHostShort((uint8_t *)(recv_ret1.c_str() + 2), (char *)(&ret_len2));
  if (ret_len2 != 0x02 && ret_len2 != 0x06) {
    int len = client_->GetAvailableBytesLen();
    if (len > 0) {
      std::string temp(len, 'x');
      client_->SyncRecv(temp, len);
    }
    LOG_F(ERROR, "Get cali ret error, data len not valid:[%d]", ret_len2);
    DisConnect();
    return DevAckError;
  }
  // get ret part 2
  std::string recv_ret2(ret_len2, 'x');
  if (client_->SyncRecv(recv_ret2, ret_len2)) {
    LOG_F(ERROR, "Receive cali ret2 error:  %d", client_->GetSysErrorCode());
    DisConnect();
    return TcpRecvTimeout;
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
    if (check) {
      uint16_t val = get_check_sum(recv_ret.substr(0, recv_ret_len - 2));
      uint16_t chk_sum = uint8_t(recv_ret[recv_ret_len - 2]) << 8 |
                         uint8_t(recv_ret[recv_ret_len - 1]);
      check = (val == chk_sum);
    }
  }

  if (!check) {
    LOG_F(ERROR, "Check cali ret error: %d.", client_->GetSysErrorCode());
    DisConnect();
    return DevAckError;
  }

  // get cali data total len
  int total_bytes_len = 0;
  zvision::NetworkToHost((uint8_t *)(recv_ret.c_str() + 5),
                         (char *)(&total_bytes_len));

  const int cali_header_len = 30;
  const int chk_sum_len = 2;
  const int new_cali_pkt_data_len = 1024;
  std::string new_cali_header_str = "CAL30S+A1";
  int received_bytes_len = 0;
  std::string cali_recv_buf_left;
  while (received_bytes_len < total_bytes_len) {
    // get cali header
    std::string header_recv(cali_header_len, 'x');
    int ret = client_->SyncRecv(header_recv, cali_header_len);
    if (ret) {
      LOG_F(ERROR, "Receive calibration header error.");
      DisConnect();
      return TcpRecvTimeout;
    }

    // get cali data len
    uint16_t data_len = 0;
    NetworkToHostShort((uint8_t *)header_recv.data() +
                           (cali_header_len - chk_sum_len),
                       (char *)(&data_len));
    if (data_len - chk_sum_len < 0) {
      LOG_F(ERROR, "Check calibration header error.");
      DisConnect();
      return InvalidContent;
    }

    // get cali data and checksum
    std::string data_recv(data_len, 'x');
    ret = client_->SyncRecv(data_recv, data_len);
    if (ret) {
      LOG_F(ERROR, "Receive calibration data error.");
      DisConnect();
      return TcpRecvTimeout;
    }

    // get packet check sum
    uint16_t pkt_chk = 0;
    NetworkToHostShort((uint8_t *)data_recv.data() + (data_len - chk_sum_len),
                       (char *)(&pkt_chk));

    // get packet data
    std::string cali_data = data_recv.substr(0, data_len - chk_sum_len);
    std::string cali_pkt = header_recv + cali_data;
    // check
    uint16_t chk = get_check_sum(cali_pkt);
    if (chk != pkt_chk) {
      LOG_F(ERROR, "Check mlxs calibration data error.");
      DisConnect();
      return DevAckError;
    }

    // check cali data
    {
      unsigned char check_all_00 = 0x00;
      unsigned char check_all_ff = 0xFF;
      for (int i = 0; i < cali_data.size(); i++) {
        check_all_00 |= cali_data[i];
        check_all_ff &= cali_data[i];
      }
      if (0x00 == check_all_00) {
        LOG_F(ERROR, "Check calibration data error, data is all 0x00.");
        DisConnect();
        return InvalidContent;
      }
      if (0xFF == check_all_ff) {
        LOG_F(ERROR, "Check calibration data error, data is all 0xFF.");
        DisConnect();
        return InvalidContent;
      }
    }

    // get pkt header information
    char header_info[7] = {(char)0x00, (char)0x00, (char)0x00, (char)0x01,
                           (char)0x00, (char)0x00, (char)0x00};
    header_info[4] = header_recv.at(25);
    // regenerate cali pkts
    std::string data_buf = cali_recv_buf_left + cali_data;

    int pkt_cnt = data_buf.size() / new_cali_pkt_data_len;
    int bytes_left = data_buf.size() % new_cali_pkt_data_len;


    if (pkt_cnt > 0) {
      for (int i = 0; i < pkt_cnt; i++) {
        uint16_t cur_pkt_cnt = pkts.size();
        header_info[0] = (cur_pkt_cnt >> 8) & 0xFF;
        header_info[1] = cur_pkt_cnt & 0xFF;
        std::string new_data =
            data_buf.substr(new_cali_pkt_data_len * i, new_cali_pkt_data_len);
        std::string new_pkt = new_cali_header_str +
                              std::string(header_info, 7) +
                              new_data; // 9 + 7 + 1024
        pkts.push_back(new_pkt);
      }
    }

    // update bytes buffer left
    cali_recv_buf_left = std::string();
    if (bytes_left > 0)
      cali_recv_buf_left =
          data_buf.substr(pkt_cnt * new_cali_pkt_data_len, bytes_left);

    // check
    received_bytes_len += (cali_header_len + data_len);
    if (received_bytes_len >= total_bytes_len) {
      break;
    }
  }
  // final check
  // we should get 100/200/400
  if (pkts.size() != 100 && pkts.size() != 200 && pkts.size() != 400) {
    LOG_F(ERROR, "Get calibration data error.");
    DisConnect();
    return NotEnoughData;
  }

  return 0;
}

int LidarTools::GetML30sPlusB1DeviceCalibrationDataToFile(
    std::string filename) {
  CalibrationPackets pkts;
  int ret = GetML30sPlusB1DeviceCalibrationPackets(pkts);
  if (ret)
    return ret;

  CalibrationData cal;
  ret = LidarTools::GetDeviceCalibrationData(pkts, cal);

  if (ret)
    return ret;
  else
    return LidarTools::ExportCalibrationData(cal, filename);
}

int LidarTools::GetML30sPlusB1DeviceSN(std::string &sn) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 9;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01,
                        (char)0x00, (char)0x02, (char)0x0D,
                        (char)0x01, (char)0x00, (char)0x00};
  std::string cmd_str(cmd, send_len);
  std::string out;
  int ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret)
    return ret;

  sn = out;
  return 0;
}

int LidarTools::GetML30sPlusB1DeviceMacAddr(std::string &mac) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 9;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01,
                        (char)0x00, (char)0x02, (char)0x0D,
                        (char)0x02, (char)0x00, (char)0x00};
  std::string cmd_str(cmd, send_len);
  std::string out;
  int ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret)
    return ret;

  if (out.size() != 6) {
    LOG_F(ERROR, "Check recv data len error.");
    DisConnect();
    return NotMatched;
  }

  char cmac[256] = "";
  sprintf_s(cmac, "%02X%02X%02X%02X%02X%02X", (uint8_t)out[0], (uint8_t)out[1],
            (uint8_t)out[2], (uint8_t)out[3], (uint8_t)out[4], (uint8_t)out[5]);
  mac = std::string(cmac);
  return 0;
}

int LidarTools::GetML30sPlusB1DeviceFirmwareVersion(FirmwareVersion &version,
                                                    FirmwareVersion &ver_bak) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 9;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01,
                        (char)0x00, (char)0x02, (char)0x0D,
                        (char)0x04, (char)0x00, (char)0x00};
  std::string cmd_str(cmd, send_len);
  std::string out;
  int ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret)
    return ret;

  if (out.size() < 16) {
    LOG_F(ERROR, "Check recv data len error.");
    DisConnect();
    return NotMatched;
  }

  uint8_t *pdata = (uint8_t *)out.data();
  memcpy(version.kernel_version, pdata + 0, 4);
  memcpy(version.boot_version, pdata + 4, 4);
  memcpy(ver_bak.kernel_version, pdata + 8, 4);
  memcpy(ver_bak.boot_version, pdata + 12, 4);
  return 0;
}

int LidarTools::GetML30sPlusB1DeviceNetworkConfigurationInfo(
    DeviceConfigurationInfo &info) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 9;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01,
                        (char)0x00, (char)0x02, (char)0x0D,
                        (char)0x03, (char)0x00, (char)0x00};
  std::string cmd_str(cmd, send_len);
  std::string out;
  int ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret)
    return ret;

  if (out.size() < 27) {
    LOG_F(ERROR, "Check recv data len error.");
    DisConnect();
    return NotMatched;
  }

  uint8_t *pdata = (uint8_t *)out.data();
  uint8_t value = (*(pdata + 0));
  if (value == 0x00)
    info.dhcp_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.dhcp_enable = StateMode::StateEnable;
  else
    info.dhcp_enable = StateMode::StateDisable;

  // ip
  ResolveIpString(pdata + 1, info.device_ip);
  // subnet mask
  ResolveIpString(pdata + 5, info.subnet_mask);
  // gateway
  ResolveIpString(pdata + 9, info.gateway_addr);
  // mac
  ResolveMacAddress(pdata + 13, info.config_mac);
  // dst ip
  ResolveIpString(pdata + 19, info.destination_ip);
  // dst port
  NetworkToHost(pdata + 23, (char *)(&info.destination_port));
  return 0;
}

int LidarTools::GetML30sPlusB1DeviceConfigParamInUse(
    DeviceConfigurationInfo &info) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 9;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01,
                        (char)0x00, (char)0x02, (char)0x0D,
                        (char)0x05, (char)0x00, (char)0x00};
  std::string cmd_str(cmd, send_len);
  std::string out;
  int ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret)
    return ret;

  if (out.size() < 0x32) {
    LOG_F(ERROR, "Check recv data len error.");
    DisConnect();
    return NotMatched;
  }
  uint8_t *pdata = (uint8_t *)out.data();
  memcpy(info.version.kernel_version, pdata + 0, 4);
  memcpy(info.version.boot_version, pdata + 4, 4);
  ResolveIpString(pdata + 8, info.device_ip);
  ResolveIpString(pdata + 12, info.gateway_addr);
  ResolveIpString(pdata + 16, info.subnet_mask);
  ResolveMacAddress(pdata + 20, info.device_mac);
  ResolveIpString(pdata + 26, info.destination_ip);
  NetworkToHost(pdata + 30, (char *)&info.destination_port);

  uint8_t value = (*(pdata + 34));
  if (value == 0x00)
    info.dhcp_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.dhcp_enable = StateMode::StateEnable;
  else
    info.dhcp_enable = StateMode::StateDisable;

  value = (*(pdata + 35));
  if (value == 0x00)
    info.retro_enable = RetroMode::RetroDisable;
  else if (value == 0x01)
    info.retro_enable = RetroMode::RetroEnable;
  else
    info.retro_enable = RetroMode::RetroDisable;

  value = (*(pdata + 36));
  if (value == 0x00)
    info.delete_point_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.delete_point_enable = StateMode::StateEnable;
  else
    info.delete_point_enable = StateMode::StateDisable;

  value = (*(pdata + 37));
  if (value == 0x00)
    info.adhesion_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.adhesion_enable = StateMode::StateEnable;
  else
    info.adhesion_enable = StateMode::StateDisable;

  value = (*(pdata + 38));
  if (value == 0)
    info.downsample_mode = DownsampleMode::DownsampleNone;
  else if (value == 1)
    info.downsample_mode = DownsampleMode::Downsample_1_2;
  else if (value == 2)
    info.downsample_mode = DownsampleMode::Downsample_1_4;
  else
    info.downsample_mode = DownsampleMode::DownsampleUnknown;

  value = (*(pdata + 39));
  if (0x01 == value)
    info.echo_mode = EchoSingleFirst;
  else if (0x02 == value)
    info.echo_mode = EchoSingleStrongest;
  else if (0x04 == value)
    info.echo_mode = EchoSingleLast;
  else if (0x03 == value)
    info.echo_mode = EchoDoubleFirstStrongest;
  else if (0x05 == value)
    info.echo_mode = EchoDoubleFirstLast;
  else if (0x06 == value)
    info.echo_mode = EchoDoubleStrongestLast;
  else
    info.echo_mode = EchoUnknown;
  ;

  value = (*(pdata + 40));
  if (0x00 == value)
    info.ptp_sync_enable = StateDisable;
  else if (0x01 == value)
    info.ptp_sync_enable = StateEnable;
  else
    info.ptp_sync_enable = StateDisable;

  value = (*(pdata + 41));
  if (value == 0x00)
    info.dirty_check_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.dirty_check_enable = StateMode::StateEnable;
  else
    info.dirty_check_enable = StateMode::StateDisable;

  value = (*(pdata + 42));
  if (0x00 == value)
    info.cal_send_mode = CalSendDisable;
  else if (0x01 == value)
    info.cal_send_mode = CalSendEnable;
  else
    info.cal_send_mode = CalSendDisable;

  value = (*(pdata + 43));
  if (value == 0x00)
    info.phase_offset_mode = PhaseOffsetMode::PhaseOffsetDisable;
  else if (value == 0x01)
    info.phase_offset_mode = PhaseOffsetMode::PhaseOffsetEnable;
  else
    info.phase_offset_mode = PhaseOffsetMode::PhaseOffsetDisable;

  NetworkToHost(pdata + 44, (char *)(&info.phase_offset));

  value = (*(pdata + 48));
  if (value == 0x00)
    info.intensity_smooth_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.intensity_smooth_enable = StateMode::StateEnable;
  else
    info.intensity_smooth_enable = StateMode::StateDisable;

  value = (*(pdata + 49));
  if (value == 0x00)
    info.diag_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.diag_enable = StateMode::StateEnable;
  else
    info.diag_enable = StateMode::StateDisable;

  value = (*(pdata + 52));
  if (value == 0x00)
    info.noTarDel_enable = StateMode::StateDisable;
  else if (value == 0x01)
    info.noTarDel_enable = StateMode::StateEnable;
  else
    info.noTarDel_enable = StateMode::StateDisable;

  return 0;
}

int LidarTools::GetML30sPlusB1DeviceAlgoParamInUse(
    DeviceConfigurationInfo &info) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 9;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01,
                        (char)0x00, (char)0x02, (char)0x0D,
                        (char)0x07, (char)0x00, (char)0x00};
  std::string cmd_str(cmd, send_len);
  std::string out;
  int ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret)
    return ret;

  if (out.size() < 85) {
    LOG_F(ERROR, "Check recv data len error.");
    DisConnect();
    return NotMatched;
  }
  int data_len = out.size();
  uint8_t *pdata = (uint8_t *)out.data();
  const unsigned char *param_data = pdata + 24;
  uint16_t param_len = data_len - 24;
  uint16_t offset = 0;
  uint16_t len = 0;
  // before 24 bytes is header data
  // near point delete
  {
    NetworkToHostShort(pdata, (char *)&offset);
    NetworkToHostShort(pdata + 2, (char *)&len);
    if (len >= 1 && ((offset + len) <= param_len)) {
      const unsigned char *param_del = param_data + offset;
      if (*(param_del + 0))
        info.delete_point_enable = StateMode::StateEnable;
      else
        info.delete_point_enable = StateMode::StateDisable;
    }
  }
  // retro
  {
    NetworkToHostShort(pdata + 4, (char *)&offset);
    NetworkToHostShort(pdata + 6, (char *)&len);
    if (len >= 9 && ((offset + len) <= param_len)) {
      const unsigned char *param_retro = param_data + offset;
      if (*(param_retro + 0))
        info.retro_enable = RetroMode::RetroEnable;
      else
        info.retro_enable = RetroMode::RetroDisable;
      info.algo_param.retro_min_gray = *(param_retro + 1);
      info.algo_param.retro_min_gray_num = *(param_retro + 2);
      NetworkToHostShort(param_retro + 3,
                         (char *)(&info.algo_param.retro_del_gray_thres));
      NetworkToHostShort(param_retro + 5,
                         (char *)&info.algo_param.retro_del_gray_dis_thres);
      info.algo_param.retro_near_del_gray_thres = *(param_retro + 7);
      info.algo_param.retro_far_del_gray_thres = *(param_retro + 8);
    }
  }

  // adhesion
  {
    NetworkToHostShort(pdata + 8, (char *)&offset);
    NetworkToHostShort(pdata + 10, (char *)&len);
    if (len >= 37 && ((offset + len) <= param_len)) {
      const unsigned char *param_adhesion = param_data + offset;
      if (*(param_adhesion + 0))
        info.adhesion_enable = StateMode::StateEnable;
      else
        info.adhesion_enable = StateMode::StateDisable;

      NetworkToHost(param_adhesion + 1,
                    (char *)&info.algo_param.adhesion_angle_hor_min);
      NetworkToHost(param_adhesion + 5,
                    (char *)&info.algo_param.adhesion_angle_hor_max);
      NetworkToHost(param_adhesion + 9,
                    (char *)&info.algo_param.adhesion_angle_ver_min);
      NetworkToHost(param_adhesion + 13,
                    (char *)&info.algo_param.adhesion_angle_ver_max);
      NetworkToHost(param_adhesion + 17,
                    (char *)&info.algo_param.adhesion_angle_hor_res);
      NetworkToHost(param_adhesion + 21,
                    (char *)&info.algo_param.adhesion_angle_ver_res);
      NetworkToHost(param_adhesion + 25,
                    (char *)&info.algo_param.adhesion_diff_thres);
      NetworkToHost(param_adhesion + 29,
                    (char *)&info.algo_param.adhesion_dis_limit);
      NetworkToHost(param_adhesion + 33,
                    (char *)&info.algo_param.adhesion_min_diff);
    }
  }

  // downsample
  {
    NetworkToHostShort(pdata + 12, (char *)&offset);
    NetworkToHostShort(pdata + 14, (char *)&len);
    if (len >= 1 && ((offset + len) <= param_len)) {
      const unsigned char *param_downsample = param_data + offset;
      unsigned char downsample_flag = (*(param_downsample + 0));
      if (0x00 == downsample_flag)
        info.downsample_mode = DownsampleNone;
      else if (0x01 == downsample_flag)
        info.downsample_mode = Downsample_1_2;
      else if (0x02 == downsample_flag)
        info.downsample_mode = Downsample_1_4;
      else
        info.downsample_mode = DownsampleUnknown;
    }
  }

  // dirty detect
  {
    NetworkToHostShort(pdata + 16, (char *)&offset);
    NetworkToHostShort(pdata + 18, (char *)&len);
    if (len >= 12 && ((offset + len) <= param_len)) {
      const unsigned char *param_dirty = param_data + offset;
      if (*(param_dirty + 0))
        info.dirty_check_enable = StateEnable;
      else
        info.dirty_check_enable = StateDisable;

      if (*(param_dirty + 1))
        info.dirty_refresh_enable = StateEnable;
      else
        info.dirty_refresh_enable = StateDisable;

      NetworkToHostShort(param_dirty + 2,
                         (char *)&info.algo_param.dirty_refresh_cycle);
      NetworkToHostShort(param_dirty + 4,
                         (char *)&info.algo_param.dirty_detect_set_thre);
      NetworkToHostShort(param_dirty + 6,
                         (char *)&info.algo_param.dirty_detect_reset_thre);
      NetworkToHostShort(param_dirty + 8,
                         (char *)&info.algo_param.dirty_detect_inner_thre);
      NetworkToHostShort(param_dirty + 10,
                         (char *)&info.algo_param.dirty_detect_outer_thre);
    }
  }

  // intensity
  {
    NetworkToHostShort(pdata + 20, (char *)&offset);
    NetworkToHostShort(pdata + 22, (char *)&len);
    if (len >= 1 && ((offset + len) <= param_len)) {
      const unsigned char *param_intensity = param_data + offset;
      if (*(param_intensity + 0))
        info.intensity_smooth_enable = StateEnable;
      else
        info.intensity_smooth_enable = StateDisable;
    }
  }

  return 0;
}

int LidarTools::QueryExcitonDeviceConfigurationInfo(
    DeviceConfigurationInfo &info) {
  // int ret = GetMLExcitonDeviceSN(info.serial_number);
  // if (ret)
  //   return ret;

  // ret = GetMLExcitonDeviceMacAddr(info.factory_mac);
  // if (ret)
  //   return ret;

  // ret = GetMLExcitonDeviceFirmwareVersion(info.version, info.backup_version);
  // if (ret)
  //   return ret;

  // ret = GetMLExcitonDeviceNetworkConfigurationInfo(info);
  // if (ret)
  //   return ret;

  // ret = GetMLExcitonDeviceConfigParamInUse(info);
  // if (ret)
  //   return ret;

  // ret = GetMLExcitonDeviceAlgoParamInUse(info);
  // if (ret)
  //   return ret;

  info.device = DeviceType::LidarExciton;
  info.algo_param.isValid = true;
  return 0;
}

int LidarTools::QueryML30sPlusB1DeviceConfigurationInfo(
    DeviceConfigurationInfo &info) {
  int ret = GetML30sPlusB1DeviceSN(info.serial_number);
  if (ret)
    return ret;

  ret = GetML30sPlusB1DeviceMacAddr(info.factory_mac);
  if (ret)
    return ret;

  ret = GetML30sPlusB1DeviceFirmwareVersion(info.version, info.backup_version);
  if (ret)
    return ret;

  ret = GetML30sPlusB1DeviceNetworkConfigurationInfo(info);
  if (ret)
    return ret;

  ret = GetML30sPlusB1DeviceConfigParamInUse(info);
  if (ret)
    return ret;

  ret = GetML30sPlusB1DeviceAlgoParamInUse(info);
  if (ret)
    return ret;

  info.device = DeviceType::LidarMl30SA1Plus;
  info.algo_param.isValid = true;
  return 0;
}

int LidarTools::GetML30sPlusB1DeviceLog(std::string &log, bool is_temp)
{
  if (!CheckConnection())
    return -1;

  const int send_len = 9;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00, (char)0x02, (char)0x0D, (char)0x0C, (char)0x00, (char)0x00};
  if (is_temp)
  {
    cmd[4] = (char)0x03;
    cmd[6] = (char)0x0B;
  }

  std::string cmd_str(cmd, send_len);
  std::string out;
  int ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret)
    return ret;

  // get log file data length
  if (out.size() != 4)
    return InvalidContent;

  int total_log_len = 0;
  int received_log_len = 0;
  NetworkToHost((uint8_t *)out.data(), (char *)(&total_log_len));
  // read log file
  std::string str_file;
  while (received_log_len < total_log_len)
  {
    ret = GetML30sPlusB1DeviceRetData(out);
    if (ret)
      return ret;

    str_file += out;
    received_log_len += out.size();
  }
  log = str_file;
  return 0;
}
int LidarTools::GetML30sPlusB1DevicePtpConfiguration(std::string &ptp_cfg) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 9;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01,
                        (char)0x00, (char)0x02, (char)0x0D,
                        (char)0x0D, (char)0x00, (char)0x00};
  std::string cmd_str(cmd, send_len);
  std::string out;
  int ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret)
    return ret;

  // get ptp file data length
  if (out.size() != 4)
    return InvalidContent;

  int total_ptp_len = 0;
  int received_ptp_len = 0;
  NetworkToHost((uint8_t *)out.data(), (char *)(&total_ptp_len));
  // read log file
  std::string str_file;
  while (received_ptp_len < total_ptp_len) {
    ret = GetML30sPlusB1DeviceRetData(out);
    if (ret)
      return ret;

    str_file += out;
    received_ptp_len += out.size();
  }
  ptp_cfg = str_file;
  return 0;
}

int LidarTools::GetML30sPlusB1DevicePtpConfigurationToFile(
    std::string &save_file_name) {
  std::string content = "";
  int ret = GetML30sPlusB1DevicePtpConfiguration(content);
  if (ret)
    return ret;
  else {
    std::ofstream out(save_file_name, std::ios::out | std::ios::binary);
    if (out.is_open()) {
      out.write(content.c_str(), content.size());
      out.close();
      return 0;
    } else
      return OpenFileError;
  }
}

int LidarTools::GenerateML30sPlusB1FilePacket(uint16_t id,
                                              const std::string &data,
                                              uint8_t cmd_type,
                                              uint8_t param_type,
                                              std::string &out) {
  const int chk_sum_len = 2;
  const int header_len = 7;
  // generate pkt header
  char cmd[header_len] = {(char)0xBA, (char)0x00, (char)0x00, (char)0x00,
                          (char)0x00, cmd_type,   param_type};
  cmd[1] = (char)((id >> 8) & 0xFF);
  cmd[2] = (char)(id & 0xFF);
  uint16_t len = data.size() + chk_sum_len;
  cmd[3] = (char)((len >> 8) & 0xFF);
  cmd[4] = (char)(len & 0xFF);
  std::string pkt_header(cmd, header_len);

  // get pkt check sum
  std::string pkt = pkt_header + data;
  uint16_t chk_sum = get_check_sum(pkt);
  char chk_str[chk_sum_len] = {(char)((chk_sum >> 8) & 0xFF),
                               (char)(chk_sum & 0xFF)};
  out = pkt + std::string(chk_str, chk_sum_len);
  return 0;
}

int LidarTools::ML30sPlusB1FirmwareUpdate(std::string &filename,
                                          ProgressCallback cb, bool isBak) {
  if (!CheckConnection())
    return TcpConnTimeout;

  // get file size
  std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
  if (!in.is_open()) {
    LOG_F(ERROR, "Open file error.");
    return OpenFileError;
  }
  std::streampos end = in.tellg();
  int size = static_cast<int>(end);
  in.close();

  // generate cmd
  const int send_len = 13;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x06, (char)0x01, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00};
  if (isBak)
    cmd[6] = (char)0x02;

  HostToNetwork((const unsigned char *)&size, cmd + 7);
  std::string cmd_str(cmd, send_len);

  // send cmd and get ret, ignore out(empty)
  std::string out;
  int ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret)
    return ret;

  // transfer, erase flash, write
  const int pkt_len = 1024;
  int start_percent = 10;
  int block_total = size / pkt_len;
  if (0 != (size % pkt_len))
    block_total += 1;
  int step_per_percent = block_total / 30;

  std::ifstream idata(filename, std::ios::in | std::ios::binary);
  char fw_data[pkt_len] = {0x00};
  int readed = 0;

  // data transfer
  if (idata.is_open()) {
    for (readed = 0; readed < block_total; readed++) {
      int read_len = pkt_len;
      if ((readed == (block_total - 1)) && (0 != (size % pkt_len)))
        read_len = size % pkt_len;
      idata.read(fw_data, read_len);
      std::string fw(fw_data, read_len);

      // generate packet
      std::string packet;
      GenerateML30sPlusB1FilePacket(readed + 1, fw, 0x01, 0x01, packet);

      if (client_->SyncSend(packet, packet.size())) {
        break;
      }
      if (0 == (readed % step_per_percent)) {
        cb(start_percent++, this->device_ip_.c_str());
      }

      // sleep
      std::this_thread::sleep_for(std::chrono::microseconds(210));
    }
    idata.close();
  }
  if (readed != block_total) {
    client_->Close();
    return TcpSendTimeout;
  }

  // get ret
  std::string str_ret;
  ret = GetML30sPlusB1DeviceRet(str_ret);
  if (ret != 0) {
    DisConnect();
    return ret;
  }

  // check ret
  ret = CheckML30sPlusB1DeviceRet(str_ret);
  if (ret != 0) {
    DisConnect();
    return ret;
  }
  LOG_F(2, "Data transfer ok...");

  // waitting for step 2
  bool ok = false;
  while (1) {
    if (GetML30sPlusB1DeviceRetData(str_ret) != 0 || str_ret.size() != 1) {
      ok = false;
      break;
    }
    unsigned char step = *((uint8_t *)str_ret.data());
    cb(40 + int((double)step / 3.3), this->device_ip_.c_str());
    if (100 <= step) {
      ok = true;
      break;
    }
  }

  if (!ok) {
    LOG_F(ERROR, "Waiting for erase flash failed...");
    DisConnect();
    return TcpRecvTimeout;
  }
  LOG_F(2, "Waiting for erase flash ok...");

  // waitting for step 3
  while (1) {
    if (GetML30sPlusB1DeviceRetData(str_ret) != 0 || str_ret.size() != 1) {
      ok = false;
      break;
    }
    unsigned char step = *((uint8_t *)str_ret.data());
    cb(70 + int((double)step / 3.3), this->device_ip_.c_str());
    if (100 <= step) {
      ok = true;
      break;
    }
  }

  if (!ok) {
    LOG_F(ERROR, "Waiting for write flash failed...");
    DisConnect();
    return TcpRecvTimeout;
  }
  LOG_F(2, "Waiting for write flash ok...");

  // recv ret
  ret = GetML30sPlusB1DeviceRet(str_ret);
  if (ret != 0) {
    DisConnect();
    return ret;
  }

  // check ret
  ret = CheckML30sPlusB1DeviceRet(str_ret);
  if (ret != 0) {
    DisConnect();
    return ret;
  }

  return 0;
}

int LidarTools::SetML30sPlusB1DeviceStaticIpAddress(std::string ip) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 13;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x06, (char)0x02, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00};

  if (!AssembleIpString(ip, cmd + 7))
    return InvalidParameter;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceGatewayAddress(std::string addr) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 13;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x06, (char)0x02, (char)0x02, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00};

  if (!AssembleIpString(addr, cmd + 7))
    return InvalidParameter;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceSubnetMaskAddress(std::string addr) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 13;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x06, (char)0x02, (char)0x03, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00};

  if (!AssembleIpString(addr, cmd + 7))
    return InvalidParameter;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceMacAddress(std::string mac) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 15;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x08, (char)0x02, (char)0x04, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00};
  if (!AssembleMacAddress(mac, cmd + 7))
    return InvalidParameter;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceUdpDestinationIpAddress(std::string ip) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 13;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x06, (char)0x02, (char)0x05, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00};

  if (!AssembleIpString(ip, cmd + 7))
    return InvalidParameter;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceUdpDestinationPort(int port) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 13;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x06, (char)0x02, (char)0x06, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00};

  AssemblePort(port, cmd + 7);

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceDHCPEnable(bool en) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x02, (char)0x07, (char)0x00,
                        (char)0x00, (char)0x00};

  if (en)
    cmd[7] = 0x01;
  else
    cmd[7] = 0x00;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceAlgorithmEnable(AlgoType tp, bool en) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00, (char)0x00};

  cmd[7] = en;
  if (tp == AlgoDeleteClosePoints) {
    cmd[5] = 0x03;
    cmd[6] = 0x01;
  } else if (tp == AlgoRetro) {
    cmd[5] = 0x04;
    cmd[6] = 0x01;
  } else if (tp == AlgoAdhesion) {
    cmd[2] = 0x00;
    cmd[4] = 0x03;
    cmd[5] = 0x05;
    cmd[6] = 0x11;
  } else if (tp == AlgoDirtyCheck) {
    cmd[5] = 0x07;
    cmd[6] = 0x01;
  } else if (tp == AlgoIntensitySmooth) {
    cmd[5] = 0x0F;
    cmd[6] = 0x01;
  } else if (tp == AlgoNoTarDel) {
    cmd[4] = 0x03;
    cmd[5] = 0x16;
    cmd[6] = 0x01;
  } 
  else
    return NotSupport;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceRetroParam(RetroParam tp, int value) {
  if (!CheckConnection())
    return TcpConnTimeout;

  std::string data;
  uint8_t cmd_tp = 0;
  uint8_t param_tp = 0;
  uint8_t val_8bit = value & 0xFF;
  switch (tp) {
  case zvision::RetroMinGray:
    data.resize(1, (char)val_8bit);
    cmd_tp = 0x04;
    param_tp = 0x03;
    break;
  case zvision::RetroMinGrayNum:
    data.resize(1, (char)val_8bit);
    cmd_tp = 0x04;
    param_tp = 0x02;
    break;
  case zvision::RetroDelGrayThres: {
    char str[2] = {(char)((value >> 8) & 0xFF), (char)(value & 0xFF)};
    data = std::string(str, 2);
    cmd_tp = 0x04;
    param_tp = 0x04;
    break;
  }
  case zvision::RetroDisThres: {
    char str[2] = {(char)((value >> 8) & 0xFF), (char)(value & 0xFF)};
    data = std::string(str, 2);
    cmd_tp = 0x04;
    param_tp = 0x05;
    break;
  }
  case zvision::RetroLowRangeThres:
    data.resize(1, (char)val_8bit);
    cmd_tp = 0x04;
    param_tp = 0x06;
    break;
  case zvision::RetroHighRangeThres:
    data.resize(1, (char)val_8bit);
    cmd_tp = 0x04;
    param_tp = 0x07;
    break;
  case zvision::RetroParamUnknown:
  default:
    return NotSupport;
  }
  std::string packet;
  GenerateML30sPlusB1FilePacket(0x01, data, cmd_tp, param_tp, packet);

  std::string out;
  return GetML30sPlusB1DeviceQueryString(packet, out);
}

int LidarTools::SetML30sPlusB1DeviceAdhesionParamParam(AdhesionParam tp,
                                                       float value) {
  if (!CheckConnection())
    return TcpConnTimeout;

  uint8_t cmd_tp = 0x05;
  uint8_t param_tp = 0;
  char values[4] = {(char)0x00, (char)0x00, (char)0x00, (char)0x00};
  int ival = value;
  switch (tp) {
  case zvision::MinimumHorizontalAngleRange:
    HostToNetwork((uint8_t *)(&ival), values);
    param_tp = 0x02;
    break;
  case zvision::MaximumHorizontalAngleRange:
    HostToNetwork((uint8_t *)(&ival), values);
    param_tp = 0x03;
    break;
  case zvision::MinimumVerticalAngleRange:
    HostToNetwork((uint8_t *)(&ival), values);
    param_tp = 0x04;
    break;
  case zvision::MaximumVerticalAngleRange:
    HostToNetwork((uint8_t *)(&ival), values);
    param_tp = 0x05;
    break;
  case zvision::HorizontalAngleResolution:
    HostToNetwork((uint8_t *)(&value), values);
    param_tp = 0x06;
    break;
  case zvision::VerticalAngleResolution:
    HostToNetwork((uint8_t *)(&value), values);
    param_tp = 0x07;
    break;
  case zvision::DeletePointThreshold:
    HostToNetwork((uint8_t *)(&value), values);
    param_tp = 0x08;
    break;
  case zvision::MaximumProcessingRange:
    HostToNetwork((uint8_t *)(&value), values);
    param_tp = 0x09;
    break;
  case zvision::NearFarPointDiff:
    HostToNetwork((uint8_t *)(&value), values);
    param_tp = 0x0A;
    break;
  case zvision::AdhesionParamUnknown:
  default:
    return NotSupport;
  }

  std::string data(values, 4);
  std::string packet;
  GenerateML30sPlusB1FilePacket(0x01, data, cmd_tp, param_tp, packet);

  std::string out;
  return GetML30sPlusB1DeviceQueryString(packet, out);
}

int LidarTools::SetML30sPlusB1DeviceDownsampleMode(DownsampleMode mode) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x06, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00};

  if (DownsampleNone == mode)
    cmd[7] = 0x00;
  else if (Downsample_1_2 == mode)
    cmd[7] = 0x01;
  else if (Downsample_1_4 == mode)
    cmd[7] = 0x02;
  else
    return ReturnCode::InvalidParameter;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceDirtyEnable(bool en) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x07, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00};

  if (en)
    cmd[7] = 0x01;
  else
    cmd[7] = 0x00;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceIntensitySmoothEnable(bool en) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x0F, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00};

  if (en)
    cmd[7] = 0x01;
  else
    cmd[7] = 0x00;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceEchoMode(EchoMode mode) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x08, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00};

  if (EchoSingleFirst == mode)
    cmd[7] = 0x01;
  else if (EchoSingleStrongest == mode)
    cmd[7] = 0x02;
  else if (EchoSingleLast == mode)
    cmd[7] = 0x04;
  else if (EchoDoubleFirstStrongest == mode)
    cmd[7] = 0x03;
  else if (EchoDoubleFirstLast == mode)
    cmd[7] = 0x05;
  else if (EchoDoubleStrongestLast == mode)
    cmd[7] = 0x06;
  else
    return ReturnCode::InvalidParameter;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DevicePtpEnable(bool en) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x09, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00};

  if (en)
    cmd[7] = 0x01;
  else
    cmd[7] = 0x00;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DevicePtpConfiguration(std::string filename) {
  if (!CheckConnection())
    return TcpConnTimeout;

  // read file
  std::string content;
  char c;
  std::ifstream inFile(filename, std::ios::in | std::ios::binary);
  if (!inFile)
    return ReturnCode::OpenFileError;
  while ((c = inFile.get()) && c != EOF)
    content.push_back(c);
  inFile.close();
  int length = content.size();

  // cmd
  const int send_len = 13;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x06, (char)0x09, (char)0x02, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00};
  AssemblePort(length, cmd + 7);

  // send cmd and check ret
  std::string cmd_str(cmd, send_len);
  std::string out;
  int ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret)
    return ret;

  // generate packet
  std::string packet;
  GenerateML30sPlusB1FilePacket(0x01, content, 0x01, 0x01, packet);
  // send file and check ret
  return GetML30sPlusB1DeviceQueryString(packet, out);
}

int LidarTools::ResetML30sPlusB1DevicePtpParam() 
{
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x09, (char)0x03, (char)0x01,
                        (char)0x00, (char)0x00};

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);

}

int LidarTools::SetML30sPlusB1DeviceFrameSyncEnable(bool en) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x0A, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00};

  if (en)
    cmd[7] = 0x01;
  else
    cmd[7] = 0x00;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceFrameSyncOffset(int value) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 13;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x06, (char)0x0A, (char)0x02, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00};
  AssemblePort(value, cmd + 7);

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceCalSendEnable(bool en) {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x0B, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00};

  if (en)
    cmd[7] = 0x01;
  else
    cmd[7] = 0x00;

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::SetML30sPlusB1DeviceeCalibrationData(std::string filename) {
  // 1. read cali data from file
  CalibrationData cal;
  int ret = 0;
  if (0 != (ret = LidarTools::ReadCalibrationData(filename, cal)))
    return ret;

  if (cal.scan_mode != ScanML30SA1Plus_160)
    return NotSupport;

  // we need trans cali data
  int lpos = cal.data.size() / 2;
  std::vector<float> cal_tmp;
  std::vector<float> cal_fov0_3 =
      std::vector<float>{std::begin(cal.data), std::begin(cal.data) + lpos};
  std::vector<float> cal_fov4_7 = std::vector<float>{
      std::begin(cal.data) + lpos, std::begin(cal.data) + cal.data.size()};
  int groups = cal.data.size() / 16;
  for (int g = 0; g < groups; g++) {
    float val = .0f;
    for (int col = 0; col < 16; col++) {
      int id = g * 16 + col;
      if (col < 8) {
        val = cal_fov0_3.at(g * 8 + col);
      } else {
        val = cal_fov4_7.at(g * 8 + col % 8);
      }
      cal.data.at(id) = val;
    }
  }

  const int cali_pkt_len = 1024;
  size_t cali_data_len = cal.data.size();
  // 2. generate calibration buffer
  std::vector<std::string> cali_pkts;
  std::string pkt(cali_pkt_len, '0');
  for (int i = 0; i < cali_data_len; i++) {
    // float to str
    char *pdata = (char *)(&cal.data[i]);
    int *paddr = (int *)pdata;
    *paddr = ntohl(*paddr);
    // update packet
    for (int j = 0; j < 4; j++)
      pkt.at(i * 4 % cali_pkt_len + j) = *(pdata + j);

    // generate packet
    if ((i + 1) % 256 == 0) {
      cali_pkts.push_back(pkt);
      pkt = std::string(cali_pkt_len, '0');
    }
  }

  if (!CheckConnection())
    return TcpConnTimeout;

  // 3. send cmd
  const int send_len = 13;
  char cmd[send_len] = {(char)0xAB, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x06, (char)0x04, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00};

  int file_len = cali_pkts.size() * cali_pkt_len;
  HostToNetwork((const unsigned char *)&file_len, cmd + 7);
  std::string cmd_str(cmd, send_len);
  std::string out;
  ret = GetML30sPlusB1DeviceQueryString(cmd_str, out);
  if (ret) {
    DisConnect();
    return ret;
  }

  // 4. send cali data to lidar
  for (int i = 0; i < cali_pkts.size(); i++) {
    std::string packet;
    GenerateML30sPlusB1FilePacket(i + 1, cali_pkts[i], 0x04, 0x01, packet);
    ret = client_->SyncSend(packet, packet.size());
    if (ret) {
      LOG_F(ERROR, "Send calibration data error, ret = %d.", ret);
      DisConnect();
      return TcpSendTimeout;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(110));
  }

  // 5. check
  std::string data_str;
  return GetML30sPlusB1DeviceRetData(data_str);
}

int LidarTools::RebootML30sPlusB1Device() {
  if (!CheckConnection())
    return TcpConnTimeout;

  const int send_len = 10;
  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x01, (char)0x00,
                        (char)0x03, (char)0x0C, (char)0x01, (char)0x01,
                        (char)0x00, (char)0x00};

  std::string cmd_str(cmd, send_len);
  std::string out;
  return GetML30sPlusB1DeviceQueryString(cmd_str, out);
}

int LidarTools::GetExcitonCompData(std::vector<float> &vector_azi,
                                   std::vector<float> &vector_ele) {
  if (!CheckConnection())
    return TcpConnTimeout;
  const int send_len = 11;

  char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x00, (char)0x00,
                        (char)0x00, (char)0x0B, (char)0x01, (char)0x00,
                        (char)0x00, (char)0x00, (char)0x00};
  std::string cmd_str(cmd, send_len);

  client_->SyncSend(cmd_str, cmd_str.size());

  uint32_t totalRecvLen = 0;
  char charRecv[7] = {0};
  client_->SyncRecv(charRecv, 7);

  if ((((int8_t)charRecv[5]) == (int8_t)0xAC) &&
      (((int8_t)charRecv[6]) == (int8_t)0x00)) {
    uint16_t recvLen = ntohs(*((uint16_t *)(charRecv + 3)) & 0xFFFF);

    char charRecvLen[8] = {0};
    client_->SyncRecv(charRecvLen, 8);
    if (recvLen == 4) {
      totalRecvLen = (charRecvLen[0] << 24) | (charRecvLen[1] << 16) |
                     (charRecvLen[2] << 8) | (charRecvLen[3]);
    }
  }

  std::string totalRecv;

  while (totalRecv.size() < totalRecvLen) {
    char charRecv[7] = {0};
    client_->SyncRecv(charRecv, 7);

    if ((((int8_t)charRecv[5]) == (int8_t)0xAC) &&
        (((int8_t)charRecv[6]) == (int8_t)0x00)) {
      uint16_t recvLen = ntohs(*((uint16_t *)(charRecv + 3)) & 0xFFFF);
      char charRecvAll[1040] = {0};
      client_->SyncRecv(charRecvAll, recvLen + 4);

      std::string strallData(charRecvAll, recvLen);

      totalRecv += strallData;
    }
  }
  // cout << totalRecv << " " << totalRecvLen << endl;
  client_->Close();

  if (totalRecv.size() ==  EXCITON_COMPDATALEN) {
    int pos = 0;
    while (pos < totalRecv.size()) {
      const char *data_ = (char *)totalRecv.c_str();

      char charAzi[4] = {0};
      char GcharAzi[4] = {0};
      strncpy(charAzi, (data_ + pos), 4);
      GcharAzi[0] = charAzi[3];
      GcharAzi[1] = charAzi[2];
      GcharAzi[2] = charAzi[1];
      GcharAzi[3] = charAzi[0];
      float aziComp = *(float *)&GcharAzi;
      vector_azi.push_back(aziComp);
      pos += 4;

      char charEle[4] = {0};
      char GcharEle[4] = {0};
      strncpy(charEle, data_ + pos, 4);
      GcharEle[0] = charEle[3];
      GcharEle[1] = charEle[2];
      GcharEle[2] = charEle[1];
      GcharEle[3] = charEle[0];
      float eleComp = *(float *)&GcharEle;
      vector_ele.push_back(eleComp);
      pos += 4;
    } 
       return 0;
  }
}
} // namespace zvision
