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


#pragma once

#include <string.h>
#include <vector>

namespace zvision {

    typedef enum LidarType{
        ML30B1,
        ML30SA1,
        ML30SPlusA1,
        ML30SPlusB1,
        MLX,
        MLXA1,
        MLXS,
        Unknown
    }LidarType;

    struct CalibrationData{
        std::vector<float> data;
        LidarType model;
    };


    typedef struct PointCalibrationData
    {
        float ele;
        float azi;
        float sin_ele;
        float cos_ele;
        float sin_azi;
        float cos_azi;
    }PointCalibrationData;

    typedef struct PointCalibrationTable
    {
        LidarType model;
        std::vector<PointCalibrationData> data;
    }PointCalibrationTable;

    class LidarTools{
        public:
            static bool CheckDeviceRet(std::string ret);
            static int GetOnlineML30SPlusB1CalibrationData(std::string ip, CalibrationData& cal);
            static int GetOnlineMLXSCalibrationData(std::string ip, CalibrationData& cal);
            static int GetOnlineCalibrationData(std::string ip, CalibrationData& cal, LidarType tp = LidarType::ML30SA1);
            static int ReadCalibrationFile(std::string filename, CalibrationData& cal);
            static void ComputeCalibrationData(CalibrationData& cal, PointCalibrationTable& cal_lut);
            static std::string GetDeviceTypeString(LidarType tp);
            static LidarType GetDeviceTypeFromTypeString(std::string tp);
            static void ComputePointLineNumber(PointCalibrationTable& cal_lut, std::vector<int>& line_numbers);

            /** \brief Set lidar retro mode.
            *   \param [in] en                true for enable,false for disable
            */
            static int SetDeviceRetroMode(std::string ip, bool en);

            /** \brief Set lidar retro mode.
            *   \param [in] en                true for enable,false for disable
            */
            static int GetDeviceRetroMode(std::string ip, bool& en);

            /** \brief Reboot lidar.
            */
            static int RebootLidar(std::string ip);

            /** \brief Get lidar state.
            */
            static bool GetLidarOnlineState(std::string ip);

            static uint16_t GetCheckSum(std::string str);
    };
}

