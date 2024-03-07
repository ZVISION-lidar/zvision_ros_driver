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


#ifndef PACKET_H_
#define PACKET_H_
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>

#include "define.h"

namespace zvision
{
    class PointCloud;

    class LidarPointsFilter;

    struct FrameTimeStamp
    {
        FrameTimeStamp()
            :timestamp(0)
            ,sys_stamp(0)
        {}

        FrameTimeStamp(double stamp_, double sys_stamp_)
        {
            timestamp = stamp_;
            sys_stamp = sys_stamp_;
        }

        double timestamp;   // from lidar udp data
        double sys_stamp;   // the time that pc received
    };

    struct LidarUdpPacket
    {
        std::string data;
        int ip;
        int ip_dst = 0xffffffff;
        /* system time (uint:s)*/
        double sys_stamp = .0f;
        /* system time (uint:ns)*/
        uint64_t stamp_ns_;
    };

    class MarkedPacket
    {
    public:
        static bool IsValidMarkedPacket(std::string& pkt);
    };

    class CalibrationPacket
    {
    public:

        static const int PACKET_LEN = 1040;

        /** \brief Packet is valid calibration udp packet or not.
        * \return true for yes, false for no.
        */
        static bool IsValidPacket(std::string& packet);

        /** \brief Get scan mode from the cal packet.
        * \return ScanMode.
        */
        static ScanMode GetScanMode(std::string& packet);

        /** \brief Get the udp sequence number from the calibration packet.
        * \return udp sequence number.
        */
        static int GetPacketSeq(std::string& packet);

        /** \brief Get the max udp sequence number by scan mode.
        * \return udp sequence number.
        */
        static int GetMaxSeq(zvision::ScanMode& mode);

        /** \brief Get device type from the cal packet.
        * \return DeviceType.
        */
        DeviceType GetDeviceType();

        /** \brief Get the udp sequence number from the calibration packet.
        * \return udp sequence number.
        */
        int GetPacketSeq();

        /** \brief Process the raw cal udp packet to float(azimuth and elevation in degree format).
        * \param[out] cal             calibration data
        */
        void ExtractData(std::vector<float>& cal);

        /** \brief Process the raw cal udp packet to float(azimuth and elevation in degree format).
        * \param[out] cal             calibration data
        */
        static void ExtractData(std::string& packet, std::vector<float>& cal);

        /** \brief cal upd packet data.
        */
        char cal_data_[1040];

    };

    class PointCloudPacket
    {
    public:

        static const int PACKET_LEN = 1304;
        static const int EXCITON_PACKET_LEN = 1320;
        static const int PACKET_LEN_IS1 = 1346;
        /** \brief Packet is valid pointcloud udp packet or not.
        * \return true for yes, false for no.
        */
        static bool IsValidPacket(std::string& packet);

        /** \brief Packet is ptp timestamp mode or not.
        * \return true for yes, false for no.
        */
        static bool IsPtpMode(std::string& packet);

        /** \brief Get packet sending interval.
        * \return packet sending interval (s).
        */
        static double GetPacketSendInterval(ScanMode mode);

        /** \brief Get device type from the pointcloud packet.
        * \return DeviceType.
        */
        static DeviceType GetDeviceType(std::string& packet);

        /** \brief Get scan mode from the pointcloud packet.
        * \return DeviceType.
        */
        static ScanMode GetScanMode(std::string& packet);

        /** \brief Get the udp sequence number from the pointcloud packet.
        * \return udp sequence number.
        */
        static int GetPacketSeq(std::string& packet);

		/** \brief Get the udp packet count in one frame pointcloud.
		* \return udp packet count( -1 for failure ).
		*/
		static int GetPacketCount(std::string& packet);

        /** \brief Get the echo count from the pointcloud packet.
        * \return eho count( 1 for single echo and 2 for dual echo ).
        */
        static int GetEchoCount(std::string& packet);

        /** \brief Get the echo count from the pointcloud packet.
        * \return eho count( 1 for single echo and 2 for dual echo ).
        */
        static int GetIS1EchoCount(std::string& packet);

        /** \brief Get the timestamp from the exciton pointcloud packet.
         * \return timestamp in second.
         */
        static double GetExcitonTimestamp(std::string& packet);

        /** \brief Get the timestamp from the pointcloud packet.
        * \return timestamp in second.
        */
        static double GetTimestamp(std::string& packet);


        /** \brief Get the timestamp from the exciton pointcloud packet.
         * \return timestamp in second.
         */
        static double GetExcitonTimestampNS(std::string& packet);

        /** \brief Get the timestamp from the exciton pointcloud packet.
         * \return timestamp in second.
         */
        static double GetIS1TimestampNS(std::string& packet);


        /** \brief Get the timestamp from the pointcloud packet.
        * \return timestamp in nano second.
        */
        static uint64_t GetTimestampNS(std::string& packet);

        /** \brief Process a pointcloud udp packet to points.
        * \param[in] packet          udp data packet
        * \param[in] cal_lut         points' cal data in sin-cos format
        * \param[in] cloud           to store the pointcloud
        * \param[in] filter
        * \param[in] stamp_ns_ptr      manu set packet timestamp(uint:ns)
        * \return 0 for ok, others for failure.
        */
        static int ProcessPacket(std::string& packet, CalibrationDataSinCosTable& cal_lut, PointCloud& cloud, LidarPointsFilter* filter = nullptr, uint64_t* stamp_ns_ptr = 0);

        static int ProcessExcitonPacket(
            std::string &packet,
            std::vector<float>& v_azi_comp_96,
            std::vector<float>& v_ele_comp_96,
            std::vector<float>& v_azi_comp_192,
            std::vector<float>& v_ele_comp_192,
            PointCloud &cloud,
            LidarPointsFilter *filter = nullptr,
            uint64_t *stamp_ns_ptr = 0
        );

        static int ProcessIS1Packet(std::string& packet, CalibrationDataSinCosTable& cal_lut, PointCloud& cloud, LidarPointsFilter* filter = nullptr, uint64_t* stamp_ns_ptr = 0);

        /** \brief Convert a point data(4 bytes) to x y z.
        * \param[in]  point           pointer to 4 bytes's point data
        * \param[out] dis             points' distance
        * \param[out] ref             points' reflectivity
        * \param[in]  dis_bit         how many bits used for distance
        * \param[in]  ref_bit         how many bits used for reflectivity
        * \return None.
        */

       static void line_96_Packet(std::string &packet, std::vector<float>& v_azi_comp, std::vector<float>& v_ele_comp, PointCloud &cloud, LidarPointsFilter *filter = nullptr, uint64_t *stamp_ns_ptr = 0);

       /** \brief Convert a point data(4 bytes) to x y z.
        * \param[in]  point           pointer to 4 bytes's point data
        * \param[out] dis             points' distance
        * \param[out] ref             points' reflectivity
        * \param[in]  dis_bit         how many bits used for distance
        * \param[in]  ref_bit         how many bits used for reflectivity
        * \return None.
        */

        static void line_192_Packet(std::string &packet, std::vector<float>& v_azi_comp, std::vector<float>& v_ele_comp, PointCloud &cloud, LidarPointsFilter *filter = nullptr, uint64_t *stamp_ns_ptr = 0);

       /** \brief Convert a point data(4 bytes) to x y z.
        * \param[in]  point           pointer to 4 bytes's point data
        * \param[out] dis             points' distance
        * \param[out] ref             points' reflectivity
        * \param[in]  dis_bit         how many bits used for distance
        * \param[in]  ref_bit         how many bits used for reflectivity
        * \return None.
        */

        static void ResolvePoint(const unsigned char* point, ReturnType& return_type, float& dis, int& ref, int dis_bit, int ref_bit);

        /** \brief pointcloud upd packet data.
        */
        char data_[1304];
        char exciton_data_[1320]; // TODO: use same buffer?
        char is1_data_[1346];


        /* system time (uint:s)*/
        double sys_timestamp = 0;
        /* system time (uint:ns)*/
        uint64_t stamp_ns_;
    };

    // New architecture protocol
    struct InternalFrameResolveInfo
    {
        int echo = 0;
        int fovs = 0;
        int lines = 0;
        int points_per_line = 0;

        int npoints = 0;
        int udp_count = 0;
        int groups_per_udp = 0;
        int points_per_group = 0;
        std::vector<int> points_offset_bytes_in_group;
        int point_size = 0;
        std::vector<int> fov_id_in_group;
    };
    typedef std::vector<BloomingPoint> BloomingPoints;

    struct BloomingFrame
    {
        BloomingPoints points;
        // udp timestamp unit:s
        double timestamp = 0;
        // system timestamp unit:s
        double sys_stamp = 0;
    };
    typedef std::shared_ptr<BloomingFrame> BloomingFramePtr;
    struct InternalPacketHeader
    {
        bool valid = false;
        ScanMode scan_mode = ScanUnknown;
        PacketType packet_type = Tp_PacketUnknown;
        uint16_t product_version = 0;
        std::string serial_number;
        int seq = -1;
        // resolve info
        InternalFrameResolveInfo resolve_info;
    };

    class InternalPacket
    {
    public:

        /** \brief packet header.
        *   Product Id:       2 bytes   (uint16_t)
        *   Product Version:  2 bytes   (uint16_t)
        *   Frame id:         2 bytes   (uint16_t)
        *   Reserved:         2 bytes   (uint16_t)
        *   Content Type:     2 bytes   (uint16_t)
        *   Block Id:         2 bytes   (uint16_t)
        *   PayLoad Length:   4 bytes   (uint32_t)
        */
        static const int PACKET_HEADER_LEN = 16;

        /** \brief packet tail.
        *   timestamp: 10 bytes
        *   reserved:   2 bytes
        *   check sum:  4 bytes   (uint32_t)
        */
        static const int PACKET_TAIL_LEN = 16;

        static const int DISTANCE_BITS = 19;

        /** distance units 0.0015m (10ps). */
        static const float DISTANCE_UNITS;

        /** \brief Get packet type.
        * \param[in] packet    udp data packet
        * \param[out] type   return packet type
        * \param[out] mode   return lidar scan mode
        */
        static void GetPacketType(const std::string& packet, PacketType& type, ScanMode& mode);

        /** \brief Get the udp sequence number from the udp packet.
        * \return udp sequence number, -1 for invalid.
        */
        static int GetPacketSeq(const std::string& packet);

       /** \brief Get the timestamp from the blooming packet.
        * \return timestamp in second.
        */
        static double GetTimestamp(const std::string& packet);

        /** \brief Get the timestamp from the blooming packet.
        */
        static void GetTimestampNS(const std::string& packet, uint64_t& s, uint32_t& ms, uint32_t& us);


        static bool GetFrameResolveInfo(const std::string& packet, InternalPacketHeader& header);

    };

    class BloomingPacket :public InternalPacket
    {

    public:
        // v2:5152 v1:3872
        static const int PACKET_LEN = 5152;
        static const int DELTA_PACKRT_US = 500;
        static const int FRAME_THRESHOLD_MS = 50;
        /** \brief Process a blooming udp packet to points.
        * \param[in] packet          udp data packet
        * \param[in] cal_lut         calibration data
        * \param[out] cloud          to store the pointcloud
        * \return 0 for ok, others for failure.
        */
        static int ProcessPacket(const std::string& packet, const zvision::CalibrationDataSinCosTable& cal_lut, BloomingFrame& frame, InternalPacketHeader* header = nullptr);

        static bool IsValidPacket(const std::string& packet);

        uint8_t data[PACKET_LEN];
        double sys_stamp = .0;
    };

}

#endif //end PACKET_H_
