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


#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <deque>
#include <unordered_map>
#include <condition_variable>
#include "define.h"
#include "protocol/packet.h"
#include "packet_source.h"

namespace zvision
{
    class UdpReceiver;
    class PcapUdpSource;
    //struct BloomingFrame;
    //struct LidarUdpPacket;
    //typedef std::shared_ptr<BloomingFrame> BloomingFramePtr;

    template <typename T>
    class SynchronizedQueue;

    class PointCloud
    {
    public:
        struct packet
        {
            packet()
                :stamp_ns(0)
            {}
            uint64_t stamp_ns;
            std::string data;
        };

        PointCloud()
            :dev_type(DeviceType::LidarUnknown)
            , scan_mode(ScanMode::ScanUnknown)
            , stamp_ns(0)
        {}

        //  device type
        DeviceType dev_type;
        std::vector<Point> points;
        // lidar packets
        std::vector<packet> packets;
        zvision::ScanMode scan_mode;
        bool is_ptp_mode = false;
        double timestamp = .0;
        double sys_stamp = .0;
        uint64_t stamp_ns;
        bool use_blooming = false;
        BloomingFramePtr blooming_frame;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief filter lidar's pointcloud data.
    */
    class LidarPointsFilter
    {
    public:
        LidarPointsFilter();
        ~LidarPointsFilter();

        /** \brief filter buckling pointcloud(only for ml30s device).
        */
        static void FilterBucklingPointCloud(PointCloud& cloud);

        /** \brief init filter parameter(only for ml30s device).
        */
        void Init(zvision::DownSampleMode mode, std::string cfg_path = "", bool is_ml30sp_b1_ep = false);

        /** \brief get filter downsample mode.
        * \return downsample mode
        */
        zvision::DownSampleMode GetDownsampleMode(zvision::ScanMode mode);
        /** \brief get filter scan mode.
        * \return scan mode
        */
        zvision::ScanMode GetScanMode();

        /** \brief get filtered points count.
        * \return points count
        */
        int GetPointsCoutFromCfgFile(int& cnt);

        /** \brief query covered point by id.
        * \return cover state
        */
        bool IsLidarPointCovered(uint32_t id);

        /** \brief func hexstring to uint8_t.
        * \return uint8_t
        */
        uint8_t hex2uint8(char h);

    private:

        zvision::DownSampleMode downsample_;
        zvision::ScanMode scan_mode_;
        std::string cfg_file_path_;
        bool init_;
        std::vector<uint8_t> cover_table_;
        int uncover_cnt_;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief PointCloudProducer for get lidar's pointcloud data.
    * \author zvision
    */
    class PointCloudProducer
    {
    public:

        typedef std::function<void(PointCloud&, int& status)> PointCloudCallback;

        /** \brief zvision PointCloudProducer constructor.
        * \param[in] data_port       lidar udp destination port.
        * \param[in] lidar_ip        lidar's ip address
        * \param[in] cal_filename    lidar's calibration file name, if empty string, using online calibration data
        * \param[in] multicast_en    enbale join multicast group
        * \param[in] mc_group_ip     multicast group ip address (224.0.0.0 -- 239.255.255.255)
        */
        PointCloudProducer(int data_port, std::string lidar_ip, std::string cal_filename = "", bool multicast_en = false, std::string mc_group_ip = "", DeviceType tp = DeviceType::LidarUnknown);


        PointCloudProducer() = delete;


        /** \brief zvision PointCloudSource destructor.
        */
        ~PointCloudProducer();


        /** \brief register pointcloud callback.
        * \param[in] cb              callback function
        */
        void RegisterPointCloudCallback(PointCloudCallback cb);

        /** \brief set pointcloud downsample mode.
        * \param[in] mode              downsample mode
        * \param[in] cfg_path          user defined config file path
        */
        void SetDownsampleMode(zvision::DownSampleMode mode, std::string cfg_path);

        /** \brief Enable internal packets processing.
        * \param[in] en              enable or disable
        */
        void SetProcessInternalPacketsEnable(bool en, bool use_lidar_time = false);

        /** \brief enable or disable saving the lidar pointcoud data .
        * \param[in] en              enable or disable
        */
        void SetPointcloudBufferEnable(bool en);

        /*start the udp handler(ThreadLoop) thread, pop up udp data in queue and process*/
        int Start();

        /*stop the udp handler(ThreadLoop) thread**/
        void Stop();

        /** \brief get pointcloud.
        * \param[out] points          to store the pointcloud data
        * \param[in]  timeout_ms      timeout to waiting for the pointcloud
        * \return 0 for success, others for failure.
        */
        int GetPointCloud(PointCloud& points, int timeout_ms);

        /** \brief get calibration data.
        * \param[out] points         store the points' calibration data(elevation and azimuth's sin cos).
        * \return 0 for success, others for failure.
        */
        int GetCalibrationData(CalibrationDataSinCosTable& cal_lut);

    protected:

        /** \brief Check the connection to device, if connection is not established, try to connect.
        * \return true for ok, false for failure.
        */
        bool CheckInit();

        /** \brief Process lidar pointcloud udp packet to pointcloud.
        * \param[in]  packet          lidar pointcloud udp packet
        */
        void ProcessLidarPacket(LidarUdpPacket& packet);

        /** \brief Process lidar internal udp packet(blooming ...).
        * \param[in]  packet          lidar pointcloud udp packet
        */
        void ProcessLidarInternalPacket(LidarUdpPacket& packet);

        /** \brief Call this function to notify to handle the new pointcloud data(store the data and notify the callback function).
        */
        void ProcessOneSweep();

        /** \brief Call this function to notify to handle the new internal pointcloud data.
*/
        void ProcessInternalOneSweep(PacketType tp);

        /*thread function: get lidar udp packet*/
        void Producer();

        /*thread function: get lidar internel udp packet*/
        void InternalProducer();

        /*thread function: get packet and process to pointcloud**/
        void Consumer();


    private:

        /** \brief Calibration data. */
        std::shared_ptr<CalibrationData> cal_;

        /** \brief store the points' calibration data(elevation and azimuth's sin cos)*/
        std::shared_ptr<CalibrationDataSinCosTable> cal_lut_;

        /** \brief store the exction angle compensation value*/
        std::vector<float> v_azi_comp_,v_ele_comp_;

        /** \brief store the lidar udp pcaket*/
        std::shared_ptr<SynchronizedQueue<LidarUdpPacket> > packets_;

        /** \brief store the lidar pointcoud data*/
        std::shared_ptr<PointCloud> points_;

        /** \brief store the lidar pointcoud data for request*/
        std::deque<std::shared_ptr<PointCloud>> pointclouds_;

        /** \brief Thread: handle the lidar packet and get the poingcloud*/
        std::shared_ptr<std::thread> consumer_;

        /** \brief Thread: receive lidar packet*/
        std::shared_ptr<std::thread> producer_;

        /** \brief receive udp data packet */
        std::shared_ptr<UdpReceiver> receiver_;

        /// internal
        bool internal_packets_enable_;
        /** \brief Thread: receive internal lidar packet*/
        std::shared_ptr<std::thread> internal_producer_;

        /** \brief receive internal udp data packet */
        std::shared_ptr<UdpReceiver> internal_receiver_;
        int internal_port_;
        std::unordered_map<PacketType, int> internal_last_seq_;
        /*internal-blooming*/
        bool match_blooming_enable_ = false;
        bool match_frame_use_lidar_time_ = false;
        std::deque<BloomingFramePtr> blooming_frame_s_;
        BloomingFramePtr blooming_frame_;

        bool use_pointcloud_buffer_;
        std::string device_ip_;
        std::string cal_filename_;
        DeviceType device_type_;
        ScanMode scan_mode_;
		DeviceType device_type_usr_;

        int last_seq_;

        LidarPointsFilter points_filter_;

        unsigned int filter_ip_;
        int data_port_;


        /** \brief lidar data udp destination ip address */
        std::string data_dst_ip_;
        bool join_multicast_;

        bool init_ok_;

        bool need_stop_;

        PointCloudCallback pointcloud_cb_;

        mutable std::mutex mutex_;
        std::condition_variable cond_;

        /** \brief pointclopud cache size.*/
        unsigned int max_pointcloud_count_;

    };


    //////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief OfflinePointCloudProducer for get lidar's pointcloud data from offline pcap file.
    * \author zvision
    */
    class OfflinePointCloudProducer
    {
    public:

        typedef std::function<void(PointCloud&, int& status)> PointCloudCallback;


        /** \brief zvision PointCloudProducer constructor.
        * \param[in] pcap_filename   offline pcap file name
        * \param[in] cal_filename    lidar's calibration file name
        * \param[in] lidar_ip        lidar's ip address
        * \param[in] data_port       lidar udp destination port.
        */
        OfflinePointCloudProducer(std::string pcap_filename, std::string cal_filename, std::string lidar_ip, int data_port);


        OfflinePointCloudProducer() = delete;


        /** \brief zvision OfflinePointCloudProducer destructor.
        */
        ~OfflinePointCloudProducer();

        /** \brief read file and analysis the pointcloud info.
        * \param[out] size            how many pointcloud in the file
        * \param[out] type            lidar type
        * \return 0 for success, others for failure.
        */
        int GetPointCloudInfo(int& size, DeviceType& type);


        /** \brief get pointcloud.
        * \param[in]  frame_number    pointcloud frame number in the pcap file
        * \param[out] points          to store the pointcloud data
        * \return 0 for success, others for failure.
        */
        int GetPointCloud(int frame_number, PointCloud& points);

        /** \brief get sin cos calibration table.
        * \param[out]  calibration data
        * \return 0 for success, others for failure.
        */
        int GetCalibrationDataSinCosTable(zvision::CalibrationDataSinCosTable&);

        /** \brief set internal frames(blooming) match method.
        * \param[in]  use_lidar_time        if value is true use lidar time or use packet timestamp.
        */
        void SetInternalFrameMatchMethod(bool use_lidar_time);

    private:
      std::shared_ptr<zvision::PcapAnalyzer> ana_;

      /*offline pcap filename */
      std::string pcap_filename_;

      std::shared_ptr<PcapUdpSource> packet_source_;

      /** \brief store the exction angle compensation value*/
      std::vector<float> v_azi_comp_96_, v_ele_comp_96_;
      std::vector<float> v_azi_comp_192_, v_ele_comp_192_;

      /*store the points' calibration data(elevation and azimuth's sin cos)*/
      std::shared_ptr<CalibrationDataSinCosTable> cal_lut_;

      /*store the lidar pointcoud data*/
      std::shared_ptr<PointCloud> points_;

      std::string cal_filename_;
      DeviceType device_type_;
      int count_;

      std::string device_ip_;

      int data_port_;

      bool init_ok_;

      PointCloudCallback pointcloud_cb_;

      bool match_frame_use_lidar_time_ = false;
    };

}

#endif //end POINT_CLOUD_H_
